const std = @import("std");
const bigToNative = std.mem.bigToNative;

const socket_can = @import("socket_can.zig");
const CanFrame = socket_can.CanFrame;

fn vescParse(T: type, slice: []const u8, scalar: f32) f32 {
    std.debug.assert(@typeInfo(T).int.bits == 8 * slice.len);
    const underlying: T = std.mem.bytesToValue(T, slice);
    return @as(f32, @floatFromInt(bigToNative(T, underlying))) / scalar;
}

pub const StatusPacket1 = struct {
    erpm: f32,
    current: f32,
    duty_cycle: f32,
    pub fn parse(can_frame: CanFrame) @This() {
        return .{
            .erpm = vescParse(i32, can_frame.data.bytes[0..4], 1),
            .current = vescParse(i16, can_frame.data.bytes[4..6], 10),
            .duty_cycle = vescParse(i16, can_frame.data.bytes[6..], 1_000),
        };
    }
};

pub const StatusPacket2 = struct {
    amp_hours: f32,
    amp_hours_chg: f32,
    pub fn parse(can_frame: CanFrame) @This() {
        return .{
            .amp_hours = vescParse(i32, can_frame.data.bytes[0..4], 10_000),
            .amp_hours_chg = vescParse(i32, can_frame.data.bytes[4..], 10_000),
        };
    }
};

pub const StatusPacket3 = struct {
    watt_hours: f32,
    watt_hours_chg: f32,
};

pub const StatusPacket4 = struct {
    temp_FET: f32,
    temp_motor: f32,
    current_in: f32,
    PID_pos: f32,
};

pub const StatusPacket5 = struct {
    tachometer: f32,
    volts_in: f32,
    pub fn parse(can_frame: CanFrame) @This() {
        return .{
            .tachometer = vescParse(i32, can_frame.data.bytes[0..4], 6),
            .volts_in = vescParse(i16, can_frame.data.bytes[4..6], 10),
        };
    }
};

pub const StatusPacket6 = struct {
    ADC1: f32,
    ADC2: f32,
    ADC3: f32,
    PPM: f32,
};

/// Sent once by a VESC when it (re)boots, payload is its hardware name in ASCII
pub const BootStatus = struct {
    hw_name: [8]u8,
    hw_name_len: u8,

    pub fn parse(can_frame: CanFrame) @This() {
        return .{
            .hw_name = can_frame.data.bytes,
            .hw_name_len = @min(can_frame.len, 8),
        };
    }

    pub fn name(self: *const @This()) []const u8 {
        return self.hw_name[0..self.hw_name_len];
    }
};

test "StatusPacket1 parses negative values" {
    var frame: CanFrame = .{ .id = .{ .vesc_id = 1, .command_type = .STATUS }, .len = 8 };
    // erpm -1000, current -12.3 A, duty -0.5
    frame.data.bytes = .{ 0xff, 0xff, 0xfc, 0x18, 0xff, 0x85, 0xfe, 0x0c };
    const status = StatusPacket1.parse(frame);
    try std.testing.expectEqual(@as(f32, -1000), status.erpm);
    try std.testing.expectApproxEqAbs(@as(f32, -12.3), status.current, 0.001);
    try std.testing.expectApproxEqAbs(@as(f32, -0.5), status.duty_cycle, 0.001);
}

test "StatusPacket5 parses negative tachometer" {
    var frame: CanFrame = .{ .id = .{ .vesc_id = 1, .command_type = .STATUS_5 }, .len = 8 };
    // tachometer -600 counts (-100 erev), 48.0 V
    frame.data.bytes = .{ 0xff, 0xff, 0xfd, 0xa8, 0x01, 0xe0, 0x00, 0x00 };
    const status = StatusPacket5.parse(frame);
    try std.testing.expectEqual(@as(f32, -100), status.tachometer);
    try std.testing.expectApproxEqAbs(@as(f32, 48.0), status.volts_in, 0.001);
}

test "BootStatus parses hardware name" {
    var frame: CanFrame = .{ .id = .{ .vesc_id = 1, .command_type = .NOTIFY_BOOT }, .len = 6 };
    frame.data.bytes = .{ '6', '0', '_', 'M', 'K', '5', 0, 0 };
    const boot = BootStatus.parse(frame);
    try std.testing.expectEqualStrings("60_MK5", boot.name());
}
