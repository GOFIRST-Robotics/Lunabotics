const std = @import("std");
const vesc_datatypes = @import("vesc_datatypes.zig");
const CommandType = vesc_datatypes.CommandType;
const packet_io = @import("packet_io.zig");
const FloatInfo = packet_io.FloatInfo;
const IntInfo = packet_io.IntInfo;
const SliceInfo = packet_io.SliceInfo;

// Control requests

pub const SetDuty = struct {
    duty: f32,

    pub const type_id = CommandType.SET_DUTY;
    pub const schema = .{
        .duty = FloatInfo{ .underlying = i32, .scalar = 100_000 },
    };
};

pub const SetCurrent = struct {
    current: f32,

    pub const type_id = CommandType.SET_CURRENT;
    pub const schema = .{
        .current = FloatInfo{ .underlying = i32, .scalar = 1_000 },
    };
};

pub const SetCurrentBrake = struct {
    current: f32,

    pub const type_id = CommandType.SET_CURRENT_BRAKE;
    pub const schema = .{
        .current = FloatInfo{ .underlying = i32, .scalar = 1_000 },
    };
};

pub const SetRPM = struct {
    rpm: f32,

    pub const type_id = CommandType.SET_RPM;
    pub const schema = .{
        .rpm = FloatInfo{ .underlying = i32, .scalar = 1 },
    };
};

pub const SetPos = struct {
    degrees: f32,

    pub const type_id = CommandType.SET_POS;
    pub const schema = .{
        .degrees = FloatInfo{ .underlying = i32, .scalar = 1_000_000 },
    };
};

// Command fragmentation

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

    pub const type_id = CommandType.PROCESS_SHORT_BUFFER;
    pub const schema = .{
        .sender_id = IntInfo{},
        .send_mode = IntInfo{},
        .data = SliceInfo{ .length = .remaining },
    };
};

pub const FillRxBuffer = struct {
    base_index: u8,
    data: []const u8,

    pub const type_id = CommandType.FILL_RX_BUFFER;
    pub const schema = .{
        .base_index = IntInfo{},
        .data = SliceInfo{ .length = .remaining },
    };
};

pub const FillRxBufferLong = struct {
    base_index: u16,
    data: []const u8,

    pub const type_id = CommandType.FILL_RX_BUFFER_LONG;
    pub const schema = .{
        .base_index = IntInfo{},
        .data = SliceInfo{ .length = .remaining },
    };
};

pub const ProcessRxBuffer = struct {
    sender_id: u8,
    send_mode: BufferSendMode,
    length: u16,
    crc: u16,

    pub const type_id = CommandType.PROCESS_RX_BUFFER;
    pub const schema = .{
        .sender_id = IntInfo{},
        .send_mode = IntInfo{},
        .length = IntInfo{},
        .crc = IntInfo{},
    };
};

const all_packets_out = [_]packet_io.PacketInfo{
    .{ .name = "set_duty", .packet = SetDuty },
    .{ .name = "set_current", .packet = SetCurrent },
    .{ .name = "set_current_brake", .packet = SetCurrentBrake },
    .{ .name = "set_rpm", .packet = SetRPM },
    .{ .name = "set_pos", .packet = SetPos },
    .{ .name = "fill_rx_buffer", .packet = FillRxBuffer },
    .{ .name = "fill_rx_buffer_long", .packet = FillRxBufferLong },
    .{ .name = "process_rx_buffer", .packet = ProcessRxBuffer },
    .{ .name = "process_short_buffer", .packet = ProcessShortBuffer },
};
const io_types = packet_io.makePacketIO(&all_packets_out);

pub const OutgoingPacketType = io_types.PacketType;
pub const OutgoingPacket = io_types.Packet;
pub const IO = io_types.IO;

test "FillRxBufferLong encoding" {
    const data = [_]u8{ 1, 2, 3, 4, 5, 6 };
    const packet = FillRxBufferLong{ .base_index = 0x3012, .data = &data };

    const expected = [_]u8{ 0x30, 0x12, 1, 2, 3, 4, 5, 6 };
    const encoded = IO.serializePacketBody(packet);
    try std.testing.expectEqualSlices(u8, &expected, encoded.data[0..encoded.len]);
}

test "FillRxBufferLong decoding" {
    const encoded = [_]u8{ 0x30, 0x12, 1, 2, 3, 4, 5, 6 };
    const packet = try IO.parsePacketBody(FillRxBufferLong, &encoded);

    const expected_data = [_]u8{ 1, 2, 3, 4, 5, 6 };
    try std.testing.expectEqual(0x3012, packet.base_index);
    try std.testing.expectEqualSlices(u8, &expected_data, packet.data);
}

test "ProcessShortBuffer encoding" {
    const data = [_]u8{ 'h', 'e', 'l', 'l', 'o' };
    const packet = ProcessShortBuffer{ .sender_id = 0x12, .send_mode = .A, .data = &data };

    const expected = [_]u8{ 0x12, 0x00, 'h', 'e', 'l', 'l', 'o' };
    const encoded = IO.serializePacketBody(packet);
    try std.testing.expectEqualSlices(u8, &expected, encoded.data[0..encoded.len]);
}

test "ProcessShortBuffer decoding" {
    const encoded = [_]u8{ 0x12, 0x00, 'h', 'e', 'l', 'l', 'o' };
    const packet = try IO.parsePacketBody(ProcessShortBuffer, &encoded);

    const expected_data = [_]u8{ 'h', 'e', 'l', 'l', 'o' };
    try std.testing.expectEqual(0x12, packet.sender_id);
    try std.testing.expectEqual(BufferSendMode.A, packet.send_mode);
    try std.testing.expectEqualSlices(u8, &expected_data, packet.data);
}
