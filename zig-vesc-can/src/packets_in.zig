const std = @import("std");
const bigToNative = std.mem.bigToNative;

const socket_can = @import("socket_can.zig");
const CanFrame = socket_can.CanFrame;

pub const ParseError = error{IncompletePacket};

const VescReader = struct {
    remaining_data: []const u8,

    pub fn init(data: []const u8) @This() {
        return .{ .remaining_data = data };
    }

    pub fn read(self: *@This(), T: type, scalar: f32) ParseError!f32 {
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

    pub fn read_slice(self: *@This(), len: usize) ParseError![]const u8 {
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

pub fn parse(T: type, data: []const u8) ParseError!T {
    var reader = VescReader.init(data);
    return T.parse(&reader);
}

pub const Status1 = struct {
    erpm: f32,
    current: f32,
    duty_cycle: f32,

    pub fn parse(reader: *VescReader) ParseError!@This() {
        return .{
            .erpm = try reader.read(i32, 1),
            .current = try reader.read(i16, 10),
            .duty_cycle = try reader.read(i16, 1_000),
        };
    }
};

pub const Status2 = struct {
    amp_hours: f32,
    amp_hours_chg: f32,

    pub fn parse(reader: *VescReader) ParseError!@This() {
        return .{
            .amp_hours = try reader.read(i32, 10_000),
            .amp_hours_chg = try reader.read(i32, 10_000),
        };
    }
};

pub const Status3 = struct {
    watt_hours: f32,
    watt_hours_chg: f32,

    pub fn parse(reader: *VescReader) ParseError!@This() {
        return .{
            .watt_hours = try reader.read(i32, 10_000),
            .watt_hours_chg = try reader.read(i32, 10_000),
        };
    }
};

pub const Status4 = struct {
    temp_FET: f32,
    temp_motor: f32,
    current_in: f32,
    PID_pos: f32,

    pub fn parse(reader: *VescReader) ParseError!@This() {
        return .{
            .temp_FET = try reader.read(i16, 10),
            .temp_motor = try reader.read(i16, 10),
            .current_in = try reader.read(i16, 10),
            .PID_pos = try reader.read(i16, 50),
        };
    }
};

pub const Status5 = struct {
    tachometer: f32,
    volts_in: f32,

    pub fn parse(reader: *VescReader) ParseError!@This() {
        return .{
            .tachometer = try reader.read(i32, 6),
            .volts_in = try reader.read(i16, 10),
        };
    }
};

pub const Status6 = struct {
    ADC1: f32,
    ADC2: f32,
    ADC3: f32,
    PPM: f32,

    pub fn parse(reader: *VescReader) ParseError!@This() {
        return .{
            .ADC1 = try reader.read(i16, 1_000),
            .ADC2 = try reader.read(i16, 1_000),
            .ADC3 = try reader.read(i16, 1_000),
            .PPM = try reader.read(i16, 1_000),
        };
    }
};

/// Sent once by a VESC when it (re)boots, payload is its hardware name in ASCII
pub const NotifyBoot = struct {
    hw_name: []const u8,

    pub fn parse(reader: *VescReader) ParseError!@This() {
        return .{
            .hw_name = try reader.read_slice(reader.remaining()),
        };
    }
};

test "Status1 parses negative values" {
    // erpm -1000, current -12.3 A, duty -0.5
    const data = [_]u8{ 0xff, 0xff, 0xfc, 0x18, 0xff, 0x85, 0xfe, 0x0c };
    var reader = VescReader.init(&data);
    const status = try Status1.parse(&reader);

    try std.testing.expectEqual(@as(f32, -1000), status.erpm);
    try std.testing.expectApproxEqAbs(@as(f32, -12.3), status.current, 0.001);
    try std.testing.expectApproxEqAbs(@as(f32, -0.5), status.duty_cycle, 0.001);
}

test "Status5 parses negative tachometer" {
    // tachometer -600 counts (-100 erev), 48.0 V
    const data = [_]u8{ 0xff, 0xff, 0xfd, 0xa8, 0x01, 0xe0, 0x00, 0x00 };
    var reader = VescReader.init(&data);
    const status = try Status5.parse(&reader);

    try std.testing.expectEqual(@as(f32, -100), status.tachometer);
    try std.testing.expectApproxEqAbs(@as(f32, 48.0), status.volts_in, 0.001);
}

test "NotifyBoot parses hardware name" {
    const data = [_]u8{ '6', '0', '_', 'M', 'K', '5' };
    var reader = VescReader.init(&data);
    const boot = try NotifyBoot.parse(&reader);

    try std.testing.expectEqualStrings("60_MK5", boot.hw_name);
}

test "Incomplete Status1 fails to parse" {
    const data = [_]u8{0x67};
    var reader = VescReader.init(&data);
    try std.testing.expectError(ParseError.IncompletePacket, Status1.parse(&reader));
}
