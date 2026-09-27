const std = @import("std");
const Io = std.Io;
const linux = std.os.linux;
const socket_can = @import("socket_can.zig");
const vesc_datatypes = @import("vesc_datatypes.zig");
const CanFrame = socket_can.CanFrame;

pub const FakeMotor = struct {
    can_fd: linux.fd_t,
    pub fn init(can_bus: []const u8) !@This() {
        var ret: @This() = undefined;
        ret.can_fd = try socket_can.openCANRaw(can_bus, null);
        return ret;
    }

    pub fn deinit(self: @This()) void {
        _ = linux.close(self.can_fd);
    }

    pub fn sendFrame(self: @This(), io: Io, frame: *const CanFrame) !void {
        try socket_can.sendRawCANFrame(io, self.can_fd, frame);
    }

    pub fn sendAtInterval(self: @This(), io: Io, frame: *const CanFrame, interval: Io.Duration) !void {
        while (true) {
            io.sleep(interval, .awake) catch return;
            self.sendFrame(io, frame) catch return;
        }
    }

    pub fn sendCount(self: @This(), io: Io, frame: *const CanFrame, count: usize) !void {
        for (0..count) |_| {
            try self.sendFrame(io, frame);
        }
    }
};

test "interval test" {
    const motor: FakeMotor = try .init("can0");
    defer motor.deinit();

    const io = std.testing.io;

    const frame: socket_can.CanFrame = .createSetCurrent(8, 763);
    var future = io.async(FakeMotor.sendAtInterval, .{ motor, io, &frame, Io.Duration.fromMilliseconds(20) });

    try io.sleep(.fromSeconds(1), .awake);

    _ = try future.cancel(io);
}

test "BCM filter" {
    const io = std.testing.io;

    const motor: FakeMotor = try .init("can0");
    defer motor.deinit();

    const can_fd = try socket_can.openCANBCM("can0", null);
    defer _ = linux.close(can_fd);

    const testing_frame: socket_can.CanFrame = .createSetDuty(13, 1984);
    try socket_can.addBCMFilterSubscription(io, can_fd, testing_frame.id);

    var future = try io.concurrent(
        FakeMotor.sendAtInterval,
        .{
            motor,
            io,
            &testing_frame,
            Io.Duration.fromMilliseconds(20),
        },
    );

    for (0..10) |_| {
        const read = try socket_can.readBCMCANFrame(io, can_fd, 1);
        try std.testing.expectEqual(testing_frame, read.can_frames[0]);
    }

    _ = try future.cancel(io);
}
