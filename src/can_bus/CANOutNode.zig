const std = @import("std");
const Io = std.Io;
const linux = std.os.linux;
const socket_can = @import("socket_can.zig");
const vesc_datatypes = @import("vesc_datatypes.zig");
const config = @import("config");
const on_jetson = config.on_jetson;

const CANOutNode = @This();

const canbus_name = config.canbus_name.*;
const logger = std.log.scoped(.can_out);

can_socket: linux.socket_t,
threaded: Io.Threaded,

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

pub fn init() @This() {
    var ret: @This() = undefined;
    ret.can_socket = socket_can.openCANRaw(&canbus_name, null) catch std.debug.panic("Failed to connected to canbus '{s}'\n", .{canbus_name});
    logger.info("successfully opened canbus '{s}', with socket {d}", .{ canbus_name, ret.can_socket });
    ret.threaded = Io.Threaded.init_single_threaded;
    return ret;
}

pub fn deinit(self: *@This()) void {
    _ = self;
}

pub fn update(self: *@This(), input: *const inputType, output: *outputType) void {
    _ = self;
    _ = output;
    _ = input;
    // const set_current_cmd = vesc_datatypes.SetCurrent.create(2, input.motor_current);
    // const set_current_cmd = vesc_datatypes.SetCurrent.create(1.0);
    // socket_can.sendRAWCANFrame(self.threaded.io(), self.can_socket, &set_current_cmd) catch @panic("Failed to send can frame");
}
