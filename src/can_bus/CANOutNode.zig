const std = @import("std");
const Io = std.Io;
const linux = std.os.linux;

const zig_vesc_can = @import("zig-vesc-can");
const can = zig_vesc_can.can;

const config = @import("config");
const on_jetson = config.on_jetson;

const CANOutNode = @This();

const canbus_name = config.canbus_name.*;
const logger = std.log.scoped(.can_out);

io: Io,
socket: can.BcmCanSocket,

pub const inputType = struct {
    motor_current: f32 = 0.0,
};
pub const outputType = struct {
    pub fn reset(self: *@This()) void {
        _ = &self;
    }
    pub fn linkBuffer(self: *@This()) void {
        _ = &self;
    }
};

pub fn init(io: Io) @This() {
    const socket = can.openSocket(&canbus_name, null) catch std.debug.panic("failed to connect to canbus '{s}'\n", .{canbus_name});

    return .{ .io = io, .socket = socket };
}

pub fn deinit(self: *@This()) void {
    _ = self;
}

pub fn update(self: *@This(), input: *const inputType, output: *outputType) void {
    _ = output;

    can.sendOnce(self.io, self.socket, 1, .{
        .set_current = .{ .current = input.motor_current },
    }) catch |err| {
        logger.err("failed to send CAN packet: {t}\n", .{err});
    };
}
