const std = @import("std");
const Io = std.Io;
const linux = std.os.linux;

const config = @import("config");
const on_jetson = config.on_jetson;
const creation = @import("../MFR/creation.zig");

const zig_vesc_can = @import("zig-vesc-can");
const can = zig_vesc_can.can;

const CANOutNode = @This();
pub const NodeName: []const u8 = "CANOutNode";

const logger = std.log.scoped(.can_out);

io: Io,
socket: can.BcmCanSocket,

pub const MotorRequests = struct {
    motor_current: f32 = 0.0,
};

pub const inputType = struct {
    motor_requests: *MotorRequests,
};

pub const outputConfigs: [0]creation.OutputConfig = .{};
pub const outputType = creation.CreateOutputType(&outputConfigs);

pub fn init(self: *CANOutNode, io: Io, socket: can.BcmCanSocket) void {
    self.io = io;
    self.socket = socket;
}

pub fn deinit(self: *@This()) void {
    _ = self;
}

pub fn update(self: *@This(), input: *const inputType, output: *outputType) void {
    _ = output;

    can.sendOnce(self.io, self.socket, 1, .{
        .set_current = .{ .current = input.motor_requests.motor_current },
    }, null) catch |err| {
        logger.err("failed to send CAN packet: {t}\n", .{err});
    };
}
