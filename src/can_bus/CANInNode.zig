const std = @import("std");
const Io = std.Io;
const linux = std.os.linux;
const zig_vesc_can = @import("zig-vesc-can");
const socket_can = zig_vesc_can.socket_can;
const vesc_datatypes = zig_vesc_can.vesc_datatypes;
const config = @import("config");
const on_jetson = config.on_jetson;

const CANInNode = @This();

const canbus_name = config.canbus_name.*;
const logger = std.log.scoped(.can_in);

io: Io,
can_socket: linux.socket_t,

pub const inputType = struct {};

pub const outputType = struct {
    test_motor_rpm: f32,
    pub fn reset(self: *@This()) void {
        _ = &self;
    }
    pub fn linkBuffer(self: *@This()) void {
        _ = &self;
    }
};

pub fn init(_: Io) @This() {
    var ret: @This() = undefined;
    ret.can_socket = socket_can.openCANBCM(&canbus_name, null) catch std.debug.panic("Failed to connected to canbus '{s}'\n", .{canbus_name});
    logger.info("successfully opened canbus '{s}', with socket {d}", .{ canbus_name, ret.can_socket });
    ret.io = undefined;

    return ret;
}

pub fn deinit(self: *@This()) void {
    linux.close(self.can_socket);
}

pub fn update(self: *@This(), input: *const inputType, output: *outputType) void {
    _ = self;
    _ = input;
    _ = output;
}
