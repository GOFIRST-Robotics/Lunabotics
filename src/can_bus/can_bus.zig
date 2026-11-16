const std = @import("std");
const Io = std.Io;

const zig_vesc_can = @import("zig-vesc-can");
const can = zig_vesc_can.can;
const TxTaskHandle = can.TxTaskHandle;
const packets = zig_vesc_can.packets;

const logger = std.log.scoped(.can_bus);

pub const CANInNode = @import("CANInNode.zig");
pub const CANOutNode = @import("CANOutNode.zig");

// CAN ID of the Jetson
pub const controller_id: u8 = 0;

// VESC IDs of each motor on the bus
pub const Motor = enum(u8) {
    // Eventually, the actual motors in the robot should be put here
    TestMotor = 1,

    pub fn byId(motor_id: u8) ?@This() {
        return std.enums.fromInt(@This(), motor_id);
    }

    pub fn id(self: @This()) u8 {
        return @backingInt(self);
    }
};
pub const motors = std.enums.values(Motor);

pub fn PerMotor(inner: type) type {
    return struct {
        values: [motors.len]inner,

        pub fn init(default_value: inner) @This() {
            return .{ .values = @splat(default_value) };
        }

        pub fn forMotor(self: *@This(), motor: Motor) *inner {
            inline for (motors, 0..) |m, i| {
                if (m == motor) return &self.values[i];
            }
            unreachable;
        }
    };
}

pub fn readNextEventOrLogFailure(io: Io, socket: can.BcmCanSocket, read_buf: *can.ReadBuffer, fail_logger: type) can.ReadError!can.VescEvent {
    var read_diag: can.ReadDiagnostic = .{};
    return can.readNextEvent(io, socket, read_buf, &read_diag) catch |err| {
        if (err == error.Canceled) {
            return error.Canceled;
        }

        fail_logger.err("failed to read CAN event: {s}\n", .{read_diag.message});
        if (read_diag.data) |data| switch (data) {
            .bcm_read_err => |e| fail_logger.err("  BCM read error: {t}\n", .{e}),
            .bad_frame => |f| {
                fail_logger.err("  frame: {any}\n", .{f.frame});
                if (f.parse_diag) |p| {
                    fail_logger.err("  decode error: {s} at byte {d}\n", .{ p.message, p.byte_position });
                }
            },
            .bad_id => |i| fail_logger.err("  ID: {any}\n", .{i}),
        };

        return err;
    };
}

// Startup initialization - detect motors that are present

const InitState = union(enum) {
    pinging: TxTaskHandle,
    finished: void,
};

const InitStateMachine = struct {
    socket: can.BcmCanSocket,
    pong_rx_handle: can.RxTaskHandle,

    states: PerMotor(InitState),

    pub fn init(io: Io, socket: can.BcmCanSocket) !@This() {
        const pong_rx_handle = try can.setupReceive(io, socket, .{
            .vesc_id = controller_id,
            .packet_type = .pong,
            .ignore_duplicates = false,
            .timeout = null,
            .rate_limit = null,
        }, null);

        var states: PerMotor(InitState) = undefined;
        for (motors) |motor| {
            states.forMotor(motor).* = .{
                .pinging = try can.setupRepeatingSend(io, socket, .{
                    .vesc_id = motor.id(),
                    .packet = .{ .ping = .{ .sender_id = controller_id } },
                    .interval = .{ .tv_sec = 0, .tv_usec = 100_000 }, // 10 Hz
                }, null),
            };
        }

        return .{ .socket = socket, .pong_rx_handle = pong_rx_handle, .states = states };
    }

    pub fn deinit(self: @This(), io: Io) void {
        can.deleteRxTask(io, self.socket, self.pong_rx_handle, null) catch {};

        for (self.states.values) |state| {
            switch (state) {
                .pinging => |handle| can.deleteTxTask(io, self.socket, handle, null) catch {},
                .finished => {},
            }
        }
    }

    pub fn allFinished(self: @This()) bool {
        for (self.states.values) |state| {
            switch (state) {
                .finished => {},
                else => return false,
            }
        }
        return true;
    }

    fn run(self: *@This(), io: Io) anyerror!void {
        while (!self.allFinished()) {
            var read_buf: can.ReadBuffer = undefined;
            const event = readNextEventOrLogFailure(io, self.socket, &read_buf, logger) catch |err| {
                switch (err) {
                    error.Canceled => return,
                    else => continue,
                }
            };

            const packet = switch (event.data) {
                .packet_received => |packet| packet,
                .packet_timeout => continue,
            };

            if (event.vesc_id == controller_id) {
                switch (packet) {
                    .pong => |pong| if (Motor.byId(pong.sender_id)) |motor| {
                        const state = self.states.forMotor(motor);
                        switch (state.*) {
                            .pinging => |handle| {
                                logger.info("received ping response from motor `{t}`", .{motor});
                                can.deleteTxTask(io, self.socket, handle, null) catch {};
                                state.* = .finished;
                            },
                            else => {},
                        }
                    },
                    else => {},
                }
            }
        }
    }
};

fn doTimeout(io: Io, duration: Io.Duration) void {
    // Will not be canceled
    io.sleep(duration, .awake) catch {};
}

// Error handling is TODO
pub fn initMotors(io: Io, socket: can.BcmCanSocket) !void {
    const Outcome = union(enum) {
        finish: anyerror!void,
        timeout: void,
    };

    const timeout: Io.Duration = .fromSeconds(5);

    logger.info("initializing motors on CAN bus (up to {f})...", .{timeout});
    var state_machine: InitStateMachine = try .init(io, socket);
    defer state_machine.deinit(io);

    var slots: [2]Outcome = undefined;
    var select: Io.Select(Outcome) = .init(io, &slots);
    try select.concurrent(.finish, InitStateMachine.run, .{ &state_machine, io });
    try select.concurrent(.timeout, doTimeout, .{ io, timeout });

    switch (try select.await()) {
        .finish => {
            logger.info("all motors initialized successfully", .{});
        },
        .timeout => {
            logger.err("not all motors finished initialization in time:", .{});
            for (motors) |motor| {
                const state = switch (state_machine.states.forMotor(motor).*) {
                    .pinging => "awaiting response to ping",
                    .finished => "finished",
                };
                logger.err("  {t}: {s}", .{ motor, state });
            }
        },
    }

    select.cancelDiscard();
}
