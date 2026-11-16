const std = @import("std");

const zig_vesc_can = @import("zig-vesc-can");
const can = zig_vesc_can.can;
const linux_can = zig_vesc_can.linux_can;

const motor_id: u8 = 1;
const status_frame_interval = linux_can.bcm_timeval{ .tv_sec = 0, .tv_usec = 20_000 }; // 20ms

pub fn main(init: std.process.Init) !void {
    const io = init.io;

    const socket = try can.openSocket("can0", null);
    defer can.closeSocket(socket);

    // Send status frames
    _ = try can.setupRepeatingSend(io, socket, .{
        .vesc_id = motor_id,
        .packet = .{ .status_1 = .{
            .erpm = 0.0,
            .current = 0.0,
            .duty_cycle = 0.0,
        } },
        .interval = status_frame_interval,
    }, null);

    // Listen for incoming packets
    _ = try can.setupReceive(io, socket, .{
        .vesc_id = motor_id,
        .packet_type = .ping,
        .ignore_duplicates = false,
        .timeout = null,
        .rate_limit = null,
    }, null);

    std.debug.print("running simulated VESC motor on CAN id {d}\n", .{motor_id});

    // Receive events
    while (true) {
        var read_buf: can.ReadBuffer = undefined;
        const event = try can.readNextEvent(io, socket, &read_buf, null);

        switch (event.data) {
            .packet_received => |packet| switch (packet) {
                .ping => |ping| {
                    try can.sendOnce(io, socket, ping.sender_id, .{ .pong = .{
                        .sender_id = motor_id,
                        .hardware_type = .vesc,
                    } }, null);
                    std.debug.print("replying to ping from ID {d}\n", .{ping.sender_id});
                },
                else => {},
            },
            .packet_timeout => {},
        }
    }
}
