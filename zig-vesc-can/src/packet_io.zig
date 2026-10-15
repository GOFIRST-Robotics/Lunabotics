const std = @import("std");
const socket_can = @import("socket_can.zig");
const CanFrame = socket_can.CanFrame;
const vesc_datatypes = @import("vesc_datatypes.zig");
const CommandType = vesc_datatypes.CommandType;

pub const FloatInfo = struct {
    underlying: type,
    scalar: f64,
};

pub const SliceInfo = struct {
    length: enum { remaining },
};

pub const PacketInfo = struct {
    name: []const u8,
    packet: type,
};

pub const IoTypes = struct {
    PacketType: type,
    Packet: type,
    IO: type,
};

pub const ParseError = error{IncompletePacket};

const VescReader = struct {
    remaining_data: []const u8,

    pub fn init(data: []const u8) @This() {
        return .{ .remaining_data = data };
    }

    pub fn readFloat(self: *@This(), T: type, scalar: f32) ParseError!f32 {
        std.debug.assert(@bitSizeOf(T) % 8 == 0);
        if (@sizeOf(T) > self.remaining_data.len) {
            return error.IncompletePacket;
        }

        const slice = self.remaining_data[0..@sizeOf(T)];
        const underlying: T = std.mem.bytesToValue(T, slice);
        const value = @as(f32, @floatFromInt(std.mem.bigToNative(T, underlying))) / scalar;

        self.remaining_data = self.remaining_data[@sizeOf(T)..];
        return value;
    }

    pub fn readSlice(self: *@This(), len: usize) ParseError![]const u8 {
        if (len > self.remaining_data.len) {
            return error.IncompletePacket;
        }

        const slice = self.remaining_data[0..len];
        self.remaining_data = self.remaining_data[len..];
        return slice;
    }

    pub fn remaining(self: @This()) usize {
        return self.remaining_data.len;
    }
};

pub const PacketData = struct {
    data: [8]u8,
    len: u8,
};

pub const VescWriter = struct {
    data: [8]u8,
    length: u8,

    pub fn init() @This() {
        return .{
            .data = @splat(0),
            .length = 0,
        };
    }

    pub fn writeFloat(self: *@This(), T: type, value: f32, scalar: f32) void {
        std.debug.assert(@bitSizeOf(T) % 8 == 0);
        const underlying: T = @trunc(@as(f64, value) * scalar);
        std.mem.writeInt(T, self.data[self.length..][0..@sizeOf(T)], underlying, .big);
        self.length += @intCast(@sizeOf(T));
    }

    pub fn writeSlice(self: *@This(), slice: []const u8) void {
        @memcpy(&self.data[(self.length)..(self.length + slice.len)], slice);
        self.length += @intCast(slice.len);
    }

    pub fn getData(self: @This()) PacketData {
        return .{ .data = self.data, .len = self.length };
    }
};

pub fn makePacketIO(packets: []const PacketInfo) IoTypes {
    var names: [packets.len][]const u8 = undefined;
    for (packets, 0..) |packet_info, i| {
        names[i] = packet_info.name;
    }

    const PacketType = make_type_enum: {
        var values: [packets.len]u8 = undefined;
        for (packets, 0..) |packet_info, i| {
            values[i] = @intFromEnum(packet_info.packet.type_id);
        }

        break :make_type_enum @Enum(u8, .exhaustive, &names, &values);
    };

    const Packet = make_union: {
        var types: [packets.len]type = undefined;
        var attrs: [packets.len]std.builtin.Type.UnionField.Attributes = undefined;
        for (packets, 0..) |packet_info, i| {
            types[i] = packet_info.packet;
            attrs[i] = .{ .@"align" = null };
        }

        break :make_union @Union(.auto, PacketType, &names, &types, &attrs);
    };

    const IO = struct {
        pub fn parsePacket(T: type, data: []const u8) ParseError!T {
            var packet: T = undefined;

            var reader = VescReader.init(data);
            const schema = T.schema;
            inline for (std.meta.fields(T)) |field| {
                const field_schema = @field(schema, field.name);

                if (@TypeOf(field_schema) == FloatInfo) {
                    @field(packet, field.name) = try reader.readFloat(field_schema.underlying, field_schema.scalar);
                } else if (@TypeOf(field_schema) == SliceInfo) {
                    const value = switch (field_schema.length) {
                        .remaining => try reader.readSlice(reader.remaining()),
                    };
                    @field(packet, field.name) = value;
                } else {
                    @panic("Invalid field info");
                }
            }

            return packet;
        }

        pub fn parsePacketByTypeId(packet_type: CommandType, data: []const u8) ParseError!?Packet {
            inline for (packets) |packet_info| {
                if (packet_type == packet_info.packet.type_id) {
                    return @unionInit(Packet, packet_info.name, try @This().parsePacket(packet_info.packet, data));
                }
            }

            // Unknown packet type
            return null;
        }

        pub fn parseFrame(frame: CanFrame) ParseError!?struct {
            source_vesc_id: u8,
            packet: Packet,
        } {
            const packet = try @This().parsePacketByTypeId(frame.id.command_type, &frame.data) orelse return null;
            return .{
                .source_vesc_id = frame.id.vesc_id,
                .packet = packet,
            };
        }

        pub fn serializeIntoFrame(dest_vesc_id: u8, packet: Packet) CanFrame {
            switch (packet) {
                inline else => |p, tag| {
                    const serialized = @This().serializePacketBody(p);
                    return .{
                        .id = .{ .vesc_id = dest_vesc_id, .command_type = @enumFromInt(@intFromEnum(tag)) },
                        .len = serialized.len,
                        .data = serialized.data,
                    };
                },
            }
        }

        pub fn serializePacketBody(packet: anytype) PacketData {
            var writer = VescWriter.init();

            const schema = @TypeOf(packet).schema;
            inline for (std.meta.fields(@TypeOf(packet))) |field| {
                const field_schema = @field(schema, field.name);

                if (@TypeOf(field_schema) == FloatInfo) {
                    writer.writeFloat(field_schema.underlying, field_schema.scalar, @field(packet, field.name));
                } else if (@TypeOf(field_schema) == SliceInfo) {
                    switch (field_schema.length) {
                        .remaining => writer.writeSlice(@field(packet, field.name)),
                    }
                } else {
                    @panic("Invalid field info");
                }
            }

            return writer.getData();
        }
    };

    return .{ .PacketType = PacketType, .Packet = Packet, .IO = IO };
}
