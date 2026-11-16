const std = @import("std");
const linux = std.os.linux;

const can = @import("can.zig");
const linux_can = @import("linux_can.h");
const packets = @import("packets.zig");

// VESC ID of the controller (this running process)
const controller_id: u8 = 0;

pub fn main(init: std.process.Init) !void {
    const io = init.io;

    const socket = try can.openSocket("can0", null);
    defer can.closeSocket(socket);
    std.debug.print("can socket is open now\n", .{});

    const controller_packets = [_]packets.PacketType{
        .fill_rx_buffer,
        .fill_rx_buffer_long,
        .process_rx_buffer,
        .process_short_buffer,
    };
    const per_motor_oneshot_packets = [_]packets.PacketType{
        .notify_boot,
        .pong,
    };
    const per_motor_periodic_packets = [_]packets.PacketType{
        .status_1,
        .status_2,
        .status_3,
        .status_4,
        .status_5,
        .status_6,
    };

    const motor_ids = [_]u8{ 1, 2, 3, 4, 5, 6 };
    const periodic_timeout = linux_can.bcm_timeval{ .tv_sec = 1, .tv_usec = 0 };
    const periodic_rate_limit = linux_can.bcm_timeval{ .tv_sec = 0, .tv_usec = 20_000 }; // 20ms

    for (controller_packets) |packet_type| {
        _ = try can.setupReceive(io, socket, .{
            .vesc_id = controller_id,
            .packet_type = packet_type,
            .ignore_duplicates = false,
            .timeout = null,
            .rate_limit = null,
        });
    }
    for (motor_ids) |motor_id| {
        for (per_motor_oneshot_packets) |packet_type| {
            _ = try can.setupReceive(io, socket, .{
                .vesc_id = motor_id,
                .packet_type = packet_type,
                .ignore_duplicates = false,
                .timeout = null,
                .rate_limit = null,
            });
        }
        for (per_motor_periodic_packets) |packet_type| {
            _ = try can.setupReceive(io, socket, .{
                .vesc_id = motor_id,
                .packet_type = packet_type,
                .ignore_duplicates = true,
                .timeout = periodic_timeout,
                .rate_limit = periodic_rate_limit,
            });
        }
    }
    std.debug.print("receive set up for all\n", .{});

    std.debug.print("listening for events\n", .{});
    while (true) {
        var read_buf: can.ReadBuffer = undefined;
        var read_diag = can.ReadDiagnostic{};
        const event = can.readNextEvent(io, socket, &read_buf, &read_diag) catch {
            std.debug.print("failed to read CAN event: {s}\n", .{read_diag.message});
            if (read_diag.data) |data| switch (data) {
                .bcm_read_err => |e| std.debug.print("  BCM read error: {t}\n", .{e}),
                .bad_frame => |f| {
                    std.debug.print("  frame: {any}\n", .{f.frame});
                    if (f.parse_diag) |p| {
                        std.debug.print("  decode error: {s} at byte {d}\n", .{ p.message, p.byte_position });
                    }
                },
                .bad_id => |i| std.debug.print("  ID: {any}\n", .{i}),
            };

            continue;
        };

        switch (event.data) {
            .packet_received => |packet| {
                std.debug.print("packet received: {any}\n", .{packet});
            },
            .packet_timeout => |packet_type| {
                std.debug.print("packet timed out: {t}\n", .{packet_type});
            },
        }
    }
}
