const std = @import("std");
const vesc_datatypes = @import("vesc_datatypes.zig");
const CommandType = vesc_datatypes.CommandType;
const packet_io = @import("packet_io.zig");
const FloatInfo = packet_io.FloatInfo;
const SliceInfo = packet_io.SliceInfo;
const socket_can = @import("socket_can.zig");
const CanFrame = socket_can.CanFrame;

pub const Status1 = struct {
    erpm: f32,
    current: f32,
    duty_cycle: f32,

    pub const type_id = CommandType.STATUS;
    pub const schema = .{
        .erpm = FloatInfo{ .underlying = i32, .scalar = 1 },
        .current = FloatInfo{ .underlying = i16, .scalar = 10 },
        .duty_cycle = FloatInfo{ .underlying = i16, .scalar = 1_000 },
    };
};

pub const Status2 = struct {
    amp_hours: f32,
    amp_hours_chg: f32,

    pub const type_id = CommandType.STATUS_2;
    pub const schema = .{
        .amp_hours = FloatInfo{ .underlying = i32, .scalar = 10_000 },
        .amp_hours_chg = FloatInfo{ .underlying = i32, .scalar = 10_000 },
    };
};

pub const Status3 = struct {
    watt_hours: f32,
    watt_hours_chg: f32,

    pub const type_id = CommandType.STATUS_3;
    pub const schema = .{
        .watt_hours = FloatInfo{ .underlying = i32, .scalar = 10_000 },
        .watt_hours_chg = FloatInfo{ .underlying = i32, .scalar = 10_000 },
    };
};

pub const Status4 = struct {
    temp_FET: f32,
    temp_motor: f32,
    current_in: f32,
    PID_pos: f32,

    pub const type_id = CommandType.STATUS_4;
    pub const schema = .{
        .temp_FET = FloatInfo{ .underlying = i16, .scalar = 10 },
        .temp_motor = FloatInfo{ .underlying = i16, .scalar = 10 },
        .current_in = FloatInfo{ .underlying = i16, .scalar = 10 },
        .PID_pos = FloatInfo{ .underlying = i16, .scalar = 50 },
    };
};

pub const Status5 = struct {
    tachometer: f32,
    volts_in: f32,

    pub const type_id = CommandType.STATUS_5;
    pub const schema = .{
        .tachometer = FloatInfo{ .underlying = i32, .scalar = 6 },
        .volts_in = FloatInfo{ .underlying = i16, .scalar = 10 },
        // Last 2 bytes are unused
    };
};

pub const Status6 = struct {
    ADC1: f32,
    ADC2: f32,
    ADC3: f32,
    PPM: f32,

    pub const type_id = CommandType.STATUS_6;
    pub const schema = .{
        .ADC1 = FloatInfo{ .underlying = i16, .scalar = 1_000 },
        .ADC2 = FloatInfo{ .underlying = i16, .scalar = 1_000 },
        .ADC3 = FloatInfo{ .underlying = i16, .scalar = 1_000 },
        .PPM = FloatInfo{ .underlying = i16, .scalar = 1_000 },
    };
};

pub const NotifyBoot = struct {
    hw_name: []const u8,

    pub const type_id = CommandType.NOTIFY_BOOT;
    pub const schema = .{
        .hw_name = SliceInfo{ .length = .remaining },
    };
};

const all_packets_in = [_]packet_io.PacketInfo{
    .{ .name = "status_1", .packet = Status1 },
    .{ .name = "status_2", .packet = Status2 },
    .{ .name = "status_3", .packet = Status3 },
    .{ .name = "status_4", .packet = Status4 },
    .{ .name = "status_5", .packet = Status5 },
    .{ .name = "status_6", .packet = Status6 },
    .{ .name = "notify_boot", .packet = NotifyBoot },
};
const io_types = packet_io.makePacketIO(&all_packets_in);

pub const IncomingPacketType = io_types.PacketType;
pub const IncomingPacket = io_types.Packet;
pub const IO = io_types.IO;

test "Status1 parses negative values" {
    // erpm -1000, current -12.3 A, duty -0.5
    var frame: CanFrame = .{ .id = .{ .vesc_id = 1, .command_type = .STATUS }, .len = 8 };
    frame.data = .{ 0xff, 0xff, 0xfc, 0x18, 0xff, 0x85, 0xfe, 0x0c };

    const id_and_packet = try IO.parseFrame(frame) orelse return error.TestUnexpectedResult;
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
    const status = try IO.parsePacketBody(Status5, &data);

    try std.testing.expectEqual(@as(f32, -100), status.tachometer);
    try std.testing.expectApproxEqAbs(@as(f32, 48.0), status.volts_in, 0.001);
}

test "NotifyBoot parses hardware name" {
    const data = [_]u8{ '6', '0', '_', 'M', 'K', '5' };
    const boot = try IO.parsePacketBody(NotifyBoot, &data);

    try std.testing.expectEqualStrings("60_MK5", boot.hw_name);
}

test "Incomplete Status1 fails to parse" {
    const data = [_]u8{0x67};
    const result = IO.parsePacketBody(Status1, &data);

    try std.testing.expectError(packet_io.ParseError.IncompletePacket, result);
}
