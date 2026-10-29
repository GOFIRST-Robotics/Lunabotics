const std = @import("std");
const linux = std.os.linux;

const can = @import("can.zig");

pub fn main(init: std.process.Init) !void {
    const io = init.io;

    const socket = try can.openSocket("can0", null);
    defer can.closeSocket(socket);
    std.debug.print("can socket is open now\n", .{});

    _ = try can.setupReceive(io, socket, .{
        .vesc_id = 1,
        .packet_type = .status_1,
        .ignore_duplicates = true,
        .timeout = .{ .tv_sec = 1, .tv_usec = 0 },
        .rate_limit = null,
    });
    std.debug.print("receive set up for Status 1\n", .{});

    _ = try can.setupRepeatingSend(io, socket, .{
        .vesc_id = 1,
        .packet = .{ .set_duty = .{ .duty = 0.5 } },
        .interval = .{ .tv_sec = 1, .tv_usec = 0 },
    });
    std.debug.print("sending duty cycle control request at 1 second intervals\n", .{});

    std.debug.print("listening for events\n", .{});
    while (true) {
        const event = try can.readNextEvent(io, socket);
        std.debug.print("event: {any}\n", .{event});
    }
}
