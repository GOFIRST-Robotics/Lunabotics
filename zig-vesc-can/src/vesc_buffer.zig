const std = @import("std");

pub const ParseError = error{ IncompletePacket, InvalidEnumValue };

pub const VescReader = struct {
    remaining_data: []const u8,

    pub fn init(data: []const u8) @This() {
        return .{ .remaining_data = data };
    }

    pub fn readInt(self: *@This(), T: type) ParseError!T {
        std.debug.assert(@bitSizeOf(T) % 8 == 0);
        if (@sizeOf(T) > self.remaining_data.len) {
            return error.IncompletePacket;
        }

        const slice = self.remaining_data[0..@sizeOf(T)];
        const big_endian: T = std.mem.bytesToValue(T, slice);
        const int_value = std.mem.bigToNative(T, big_endian);

        self.remaining_data = self.remaining_data[@sizeOf(T)..];
        return int_value;
    }

    pub fn readEnum(self: *@This(), T: type) ParseError!T {
        const IntT = @typeInfo(T).@"enum".tag_type;
        const int_value = try self.readInt(IntT);

        if (std.enums.fromInt(T, int_value)) |v| {
            return v;
        } else {
            return error.InvalidEnumValue;
        }
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

    pub fn writeInt(self: *@This(), T: type, value: T) void {
        std.debug.assert(@bitSizeOf(T) % 8 == 0);
        std.mem.writeInt(T, self.data[self.length..][0..@sizeOf(T)], value, .big);
        self.length += @intCast(@sizeOf(T));
    }

    pub fn writeEnum(self: *@This(), T: type, value: T) void {
        const int_value = @intFromEnum(value);
        const IntT = @TypeOf(int_value);
        self.writeInt(IntT, int_value);
    }

    pub fn writeFloat(self: *@This(), T: type, value: f32, scalar: f32) void {
        std.debug.assert(@bitSizeOf(T) % 8 == 0);
        const underlying: T = @trunc(@as(f64, value) * scalar);
        std.mem.writeInt(T, self.data[self.length..][0..@sizeOf(T)], underlying, .big);
        self.length += @intCast(@sizeOf(T));
    }

    pub fn writeSlice(self: *@This(), slice: []const u8) void {
        @memcpy(self.data[(self.length)..(self.length + slice.len)], slice);
        self.length += @intCast(slice.len);
    }

    pub fn getData(self: @This()) PacketData {
        return .{ .data = self.data, .len = self.length };
    }
};
