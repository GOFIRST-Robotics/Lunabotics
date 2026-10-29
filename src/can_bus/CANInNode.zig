const std = @import("std");
const Io = std.Io;
const linux = std.os.linux;

const zig_vesc_can = @import("zig-vesc-can");
const can = zig_vesc_can.can;

const config = @import("config");
const on_jetson = config.on_jetson;

const CANInNode = @This();

const canbus_name = config.canbus_name.*;
const logger = std.log.scoped(.can_in);

io: Io,
socket: can.BcmCanSocket,

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

pub fn init(io: Io) @This() {
    // TODO: Pass in from main
    const socket = can.openSocket(&canbus_name, null) catch std.debug.panic("failed to connect to canbus '{s}'\n", .{canbus_name});

    // Just read from VESC 1 for testing
    _ = can.setupReceive(io, socket, .{
        .vesc_id = 1,
        .packet_type = .status_1,
        .ignore_duplicates = true,
        .timeout = .{ .tv_sec = 1, .tv_usec = 0 },
        .rate_limit = null,
    }, null) catch std.debug.panic("failed to setup receive for status 1 packet\n", .{});

    return .{ .io = io, .socket = socket };
}

pub fn deinit(self: *@This()) void {
    _ = self;
}

pub fn update(self: *@This(), input: *const inputType, output: *outputType) void {
    _ = input;
    while (true) {
        var read_diag: can.ReadDiagnostic = .{};
        const event = can.readNextEvent(self.io, self.socket, &read_diag) catch |err| {
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
                    output.test_motor_rpm = status.erpm;
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
