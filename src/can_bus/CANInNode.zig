const std = @import("std");
const Io = std.Io;
const linux = std.os.linux;

const config = @import("config");
const on_jetson = config.on_jetson;
const creation = @import("../MFR/creation.zig");
const TeleopStateMachineNode = @import("../teleop_state_machine/TeleopStateMachineNode.zig");

const zig_vesc_can = @import("zig-vesc-can");
const can = zig_vesc_can.can;

const CANInNode = @This();
pub const NodeName: []const u8 = "CANInNode";

const logger = std.log.scoped(.can_in);

io: Io,
socket: can.BcmCanSocket,

pub const inputType = struct {};

pub const MotorStatus = struct {
    test_motor_rpm: f32 = 0.0,
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
    // Just read from VESC 1 for testing
    _ = can.setupReceive(io, socket, .{
        .vesc_id = 1,
        .packet_type = .status_1,
        .ignore_duplicates = true,
        .timeout = .{ .tv_sec = 1, .tv_usec = 0 },
        .rate_limit = null,
    }, null) catch std.debug.panic("failed to setup receive for status 1 packet\n", .{});

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
        var read_diag: can.ReadDiagnostic = .{};
        const event = can.readNextEvent(self.io, self.socket, &read_buf, &read_diag) catch |err| {
            if (err == error.Canceled) {
                return;
            }

            logger.err("failed to read CAN event: {s}\n", .{read_diag.message});
            if (read_diag.data) |data| switch (data) {
                .bcm_read_err => |e| logger.err("  BCM read error: {t}\n", .{e}),
                .bad_frame => |f| {
                    logger.err("  frame: {any}\n", .{f.frame});
                    if (f.parse_diag) |p| {
                        logger.err("  decode error: {s} at byte {d}\n", .{ p.message, p.byte_position });
                    }
                },
                .bad_id => |i| logger.err("  ID: {any}\n", .{i}),
            };
            return;
        };

        switch (event.data) {
            .packet_received => |packet| switch (packet) {
                .status_1 => |status| {
                    output.TeleopStateMachine.motor_status.* = .{ .test_motor_rpm = status.erpm };
                    return;
                },
                else => {},
            },
            .packet_timeout => |packet_type| {
                logger.info("status frame {t} timed out from VESC {d}\n", .{ packet_type, event.vesc_id });
            },
        }
    }
}
