const std = @import("std");
const can = @import("can.zig");
const codec = @import("codec.zig");

const CanFrame = can.CanFrame;
const VescCanId = can.VescCanId;
const BufReader = codec.BufReader;
const BufWriter = codec.BufWriter;

pub const PacketType = enum(u8) {
    status_1 = 9,
    status_2 = 14,
    status_3 = 15,
    status_4 = 16,
    status_5 = 27,
    status_6 = 58,
    notify_boot = 57,
    set_duty = 0,
    set_current = 1,
    set_current_brake = 2,
    set_rpm = 3,
    set_pos = 4,
    fill_rx_buffer = 5,
    fill_rx_buffer_long = 6,
    process_rx_buffer = 7,
    process_short_buffer = 8,
};

pub const Packet = union(PacketType) {
    // Status signals (VESC -> controller)
    status_1: Status1,
    status_2: Status2,
    status_3: Status3,
    status_4: Status4,
    status_5: Status5,
    status_6: Status6,
    notify_boot: NotifyBoot,

    // Control requests (controller -> VESC)
    set_duty: SetDuty,
    set_current: SetCurrent,
    set_current_brake: SetCurrentBrake,
    set_rpm: SetRPM,
    set_pos: SetPos,

    // Command fragmentation (bidirectional)
    fill_rx_buffer: FillRxBuffer,
    fill_rx_buffer_long: FillRxBufferLong,
    process_rx_buffer: ProcessRxBuffer,
    process_short_buffer: ProcessShortBuffer,
};

pub const Status1 = struct {
    erpm: f32,
    current: f32,
    duty_cycle: f32,

    pub const schema = .{
        .erpm = codec.FixedFloat{ .underlying = i32, .scalar = 1 },
        .current = codec.FixedFloat{ .underlying = i16, .scalar = 10 },
        .duty_cycle = codec.FixedFloat{ .underlying = i16, .scalar = 1_000 },
    };
};

pub const Status2 = struct {
    amp_hours: f32,
    amp_hours_chg: f32,

    pub const type_id = PacketType.status_2;
    pub const schema = .{
        .amp_hours = codec.FixedFloat{ .underlying = i32, .scalar = 10_000 },
        .amp_hours_chg = codec.FixedFloat{ .underlying = i32, .scalar = 10_000 },
    };
};

pub const Status3 = struct {
    watt_hours: f32,
    watt_hours_chg: f32,

    pub const type_id = PacketType.status_3;
    pub const schema = .{
        .watt_hours = codec.FixedFloat{ .underlying = i32, .scalar = 10_000 },
        .watt_hours_chg = codec.FixedFloat{ .underlying = i32, .scalar = 10_000 },
    };
};

pub const Status4 = struct {
    temp_FET: f32,
    temp_motor: f32,
    current_in: f32,
    PID_pos: f32,

    pub const type_id = PacketType.status_4;
    pub const schema = .{
        .temp_FET = codec.FixedFloat{ .underlying = i16, .scalar = 10 },
        .temp_motor = codec.FixedFloat{ .underlying = i16, .scalar = 10 },
        .current_in = codec.FixedFloat{ .underlying = i16, .scalar = 10 },
        .PID_pos = codec.FixedFloat{ .underlying = i16, .scalar = 50 },
    };
};

pub const Status5 = struct {
    tachometer: f32,
    volts_in: f32,

    pub const type_id = PacketType.status_5;
    pub const schema = .{
        .tachometer = codec.FixedFloat{ .underlying = i32, .scalar = 6 },
        .volts_in = codec.FixedFloat{ .underlying = i16, .scalar = 10 },
        // Last 2 bytes are unused
    };
};

pub const Status6 = struct {
    ADC1: f32,
    ADC2: f32,
    ADC3: f32,
    PPM: f32,

    pub const type_id = PacketType.status_6;
    pub const schema = .{
        .ADC1 = codec.FixedFloat{ .underlying = i16, .scalar = 1_000 },
        .ADC2 = codec.FixedFloat{ .underlying = i16, .scalar = 1_000 },
        .ADC3 = codec.FixedFloat{ .underlying = i16, .scalar = 1_000 },
        .PPM = codec.FixedFloat{ .underlying = i16, .scalar = 1_000 },
    };
};

pub const NotifyBoot = struct {
    hw_name: []const u8,

    pub const type_id = PacketType.notify_boot;
    pub const schema = .{
        .hw_name = codec.remaining_bytes,
    };
};

pub const SetDuty = struct {
    duty: f32,

    pub const type_id = PacketType.set_duty;
    pub const schema = .{
        .duty = codec.FixedFloat{ .underlying = i32, .scalar = 100_000 },
    };
};

pub const SetCurrent = struct {
    current: f32,

    pub const type_id = PacketType.set_current;
    pub const schema = .{
        .current = codec.FixedFloat{ .underlying = i32, .scalar = 1_000 },
    };
};

pub const SetCurrentBrake = struct {
    current: f32,

    pub const type_id = PacketType.set_current_brake;
    pub const schema = .{
        .current = codec.FixedFloat{ .underlying = i32, .scalar = 1_000 },
    };
};

pub const SetRPM = struct {
    rpm: f32,

    pub const type_id = PacketType.set_rpm;
    pub const schema = .{
        .rpm = codec.FixedFloat{ .underlying = i32, .scalar = 1 },
    };
};

pub const SetPos = struct {
    degrees: f32,

    pub const type_id = PacketType.set_pos;
    pub const schema = .{
        .degrees = codec.FixedFloat{ .underlying = i32, .scalar = 1_000_000 },
    };
};

pub const BufferSendMode = enum(u8) {
    // TODO: Figure out what these actually do; all I know is there's 4 of them and 0 is default
    A = 0,
    B = 1,
    C = 2,
    D = 3,
};

pub const ProcessShortBuffer = struct {
    sender_id: u8,
    send_mode: BufferSendMode,
    data: []const u8,

    pub const type_id = PacketType.process_short_buffer;
    pub const schema = .{
        .sender_id = codec.int,
        .send_mode = codec.int_enum,
        .data = codec.remaining_bytes,
    };
};

pub const FillRxBuffer = struct {
    base_index: u8,
    data: []const u8,

    pub const type_id = PacketType.fill_rx_buffer;
    pub const schema = .{
        .base_index = codec.int,
        .data = codec.remaining_bytes,
    };
};

pub const FillRxBufferLong = struct {
    base_index: u16,
    data: []const u8,

    pub const type_id = PacketType.fill_rx_buffer_long;
    pub const schema = .{
        .base_index = codec.int,
        .data = codec.remaining_bytes,
    };
};

pub const ProcessRxBuffer = struct {
    sender_id: u8,
    send_mode: BufferSendMode,
    length: u16,
    crc: u16,

    pub const type_id = PacketType.process_rx_buffer;
    pub const schema = .{
        .sender_id = codec.int,
        .send_mode = codec.int_enum,
        .length = codec.int,
        .crc = codec.int,
    };
};

pub fn parsePacketBody(T: type, data: []const u8) !T {
    const packet_codec = codec.structSchema(T.schema);
    var reader = BufReader.from(data);
    return try packet_codec.decode(T, &reader);
}

pub fn parsePacket(packet_type: PacketType, data: []const u8) !?Packet {
    const fields = std.meta.fields(Packet);
    inline for (fields) |field| {
        if (packet_type == @field(PacketType, field.name)) {
            return @unionInit(Packet, field.name, try parsePacketBody(field.type, data));
        }
    }

    // Unknown packet type
    return null;
}

pub fn parseFrame(frame: CanFrame) !?struct {
    source_vesc_id: u8,
    packet: Packet,
} {
    const vesc_id = VescCanId.fromCanId(frame.id) orelse return null;
    const packet_type = std.enums.fromInt(PacketType, vesc_id.packet_type) orelse return null;
    const packet = try parsePacket(packet_type, &frame.data) orelse return null;

    return .{
        .source_vesc_id = vesc_id.vesc_id,
        .packet = packet,
    };
}

pub const PacketData = struct {
    data: [8]u8,
    len: u8,
};

pub fn serializePacketBody(packet: anytype) !PacketData {
    const packet_codec = codec.structSchema(@TypeOf(packet).schema);

    var data: PacketData = undefined;
    var writer = codec.BufWriter.into(&data.data);
    try packet_codec.encode(packet, &writer);
    data.len = @intCast(writer.writtenLength());

    return data;
}

pub fn serializeIntoFrame(dest_vesc_id: u8, packet: Packet) !CanFrame {
    switch (packet) {
        inline else => |body, packet_type| {
            const serialized = try serializePacketBody(body);
            return .{
                .id = (VescCanId{ .vesc_id = dest_vesc_id, .packet_type = @intFromEnum(packet_type) }).toCanId(),
                .len = serialized.len,
                .data = serialized.data,
            };
        },
    }
}

test "Status1 parses negative values" {
    // erpm -1000, current -12.3 A, duty -0.5
    const id: VescCanId = .{ .vesc_id = 1, .packet_type = @intFromEnum(PacketType.status_1) };
    var frame: CanFrame = .{ .id = id.toCanId(), .len = 8 };
    frame.data = .{ 0xff, 0xff, 0xfc, 0x18, 0xff, 0x85, 0xfe, 0x0c };

    const id_and_packet = try parseFrame(frame) orelse return error.TestUnexpectedResult;
    try std.testing.expectEqual(id_and_packet.source_vesc_id, 1);
    const packet = id_and_packet.packet;
    const status = switch (packet) {
        .status_1 => |status| status,
        else => return error.TestUnexpectedResult,
    };

    try std.testing.expectEqual(@as(f32, -1000), status.erpm);
    try std.testing.expectApproxEqAbs(@as(f32, -12.3), status.current, 0.001);
    try std.testing.expectApproxEqAbs(@as(f32, -0.5), status.duty_cycle, 0.001);
}

test "Status5 parses negative tachometer" {
    // tachometer -600 counts (-100 erev), 48.0 V
    const data = [_]u8{ 0xff, 0xff, 0xfd, 0xa8, 0x01, 0xe0, 0x00, 0x00 };
    const status = try parsePacketBody(Status5, &data);

    try std.testing.expectEqual(@as(f32, -100), status.tachometer);
    try std.testing.expectApproxEqAbs(@as(f32, 48.0), status.volts_in, 0.001);
}

test "NotifyBoot parses hardware name" {
    const data = [_]u8{ '6', '0', '_', 'M', 'K', '5' };
    const boot = try parsePacketBody(NotifyBoot, &data);

    try std.testing.expectEqualStrings("60_MK5", boot.hw_name);
}

test "Incomplete Status1 fails to parse" {
    const data = [_]u8{0x67};
    const result = parsePacketBody(Status1, &data);

    try std.testing.expectError(error.EndOfStream, result);
}

test "FillRxBufferLong encoding" {
    const data = [_]u8{ 1, 2, 3, 4, 5, 6 };
    const packet = FillRxBufferLong{ .base_index = 0x3012, .data = &data };

    const expected = [_]u8{ 0x30, 0x12, 1, 2, 3, 4, 5, 6 };
    const encoded = try serializePacketBody(packet);
    try std.testing.expectEqualSlices(u8, &expected, encoded.data[0..encoded.len]);
}

test "FillRxBufferLong decoding" {
    const encoded = [_]u8{ 0x30, 0x12, 1, 2, 3, 4, 5, 6 };
    const packet = try parsePacketBody(FillRxBufferLong, &encoded);

    const expected_data = [_]u8{ 1, 2, 3, 4, 5, 6 };
    try std.testing.expectEqual(0x3012, packet.base_index);
    try std.testing.expectEqualSlices(u8, &expected_data, packet.data);
}

test "ProcessShortBuffer encoding" {
    const data = [_]u8{ 'h', 'e', 'l', 'l', 'o' };
    const packet = ProcessShortBuffer{ .sender_id = 0x12, .send_mode = .A, .data = &data };

    const expected = [_]u8{ 0x12, 0x00, 'h', 'e', 'l', 'l', 'o' };
    const encoded = try serializePacketBody(packet);
    try std.testing.expectEqualSlices(u8, &expected, encoded.data[0..encoded.len]);
}

test "ProcessShortBuffer decoding" {
    const encoded = [_]u8{ 0x12, 0x00, 'h', 'e', 'l', 'l', 'o' };
    const packet = try parsePacketBody(ProcessShortBuffer, &encoded);

    const expected_data = [_]u8{ 'h', 'e', 'l', 'l', 'o' };
    try std.testing.expectEqual(0x12, packet.sender_id);
    try std.testing.expectEqual(BufferSendMode.A, packet.send_mode);
    try std.testing.expectEqualSlices(u8, &expected_data, packet.data);
}
