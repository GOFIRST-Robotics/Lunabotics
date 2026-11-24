const std = @import("std");
const Io = std.Io;
const linux = std.os.linux;

const config = @import("config");
const on_jetson = config.on_jetson;
const creation = @import("../MFR/creation.zig");
const TeleopStateMachineNode = @import("../teleop_state_machine/TeleopStateMachineNode.zig");

const zig_vesc_can = @import("zig-vesc-can");
const can = zig_vesc_can.can;
const packets = zig_vesc_can.packets;

const can_bus = @import("can_bus.zig");
const Motor = can_bus.Motor;
const PerMotor = can_bus.PerMotor;

const CANInNode = @This();
pub const NodeName: []const u8 = "CANInNode";

const logger = std.log.scoped(.can_in);

io: Io,
socket: can.BcmCanSocket,

pub const inputType = struct {};

pub const MotorStatus = struct {
    // May be null if the frame has not been received for too long
    status_1: ?packets.Status1 = null,
    status_2: ?packets.Status2 = null,
    status_3: ?packets.Status3 = null,
    status_4: ?packets.Status4 = null,
    status_5: ?packets.Status5 = null,
    status_6: ?packets.Status6 = null,
};

// This only exists because creation requires fields to be initializable as .{}
pub const AllMotorStatuses = struct {
    all: PerMotor(MotorStatus) = .init(.{}),

    pub fn forMotor(self: *@This(), motor: Motor) *MotorStatus {
        return self.all.forMotor(motor);
    }
};

pub const outputConfigs = [_]creation.OutputConfig{
    .{
        .output_node = TeleopStateMachineNode,
        .output_field = &.{.{
            .field = .motor_status,
            .buffering_type = .double_buffered,
        }},
    },
};

pub const outputType = creation.CreateOutputType(&outputConfigs);

pub fn init(self: *CANInNode, io: Io, socket: can.BcmCanSocket) void {
    const status_packets = [_]packets.PacketType{
        .status_1,
        .status_2,
        .status_3,
        .status_4,
        .status_5,
        .status_6,
    };

    // Setup receive for all status signals from all motors
    for (can_bus.motors) |motor| {
        for (status_packets) |packet_type| {
            _ = can.setupReceive(io, socket, .{
                .vesc_id = motor.id(),
                .packet_type = packet_type,
                .ignore_duplicates = true,
                .timeout = .{ .tv_sec = 1, .tv_usec = 0 },
                .rate_limit = .{ .tv_sec = 1, .tv_usec = 20_000 }, // 20 ms
            }, null) catch std.debug.panic("failed to setup packet receive", .{});
        }
    }

    self.io = io;
    self.socket = socket;
}

pub fn deinit(self: *@This()) void {
    _ = self;
}

pub fn update(self: *@This(), input: *const inputType, output: *outputType) void {
    _ = input;
    while (true) {
        var read_buf: can.ReadBuffer = undefined;
        const event = can_bus.readNextEventOrLogFailure(self.io, self.socket, &read_buf, logger) catch |err| {
            switch (err) {
                error.Canceled => return,
                else => continue,
            }
        };

        if (Motor.byId(event.vesc_id)) |motor| {
            const motor_status = output.TeleopStateMachine.motor_status.forMotor(motor);
            switch (event.data) {
                .packet_received => |packet| switch (packet) {
                    .status_1 => |s| motor_status.status_1 = s,
                    .status_2 => |s| motor_status.status_2 = s,
                    .status_3 => |s| motor_status.status_3 = s,
                    .status_4 => |s| motor_status.status_4 = s,
                    .status_5 => |s| motor_status.status_5 = s,
                    .status_6 => |s| motor_status.status_6 = s,
                    else => {},
                },
                .packet_timeout => |packet_type| {
                    logger.err("CAN packet `{t}` timed out on motor `{t}`", .{ packet_type, motor });
                    switch (packet_type) {
                        .status_1 => motor_status.status_1 = null,
                        .status_2 => motor_status.status_2 = null,
                        .status_3 => motor_status.status_3 = null,
                        .status_4 => motor_status.status_4 = null,
                        .status_5 => motor_status.status_5 = null,
                        .status_6 => motor_status.status_6 = null,
                        else => {},
                    }
                },
            }
            return;
        }
    }
}
