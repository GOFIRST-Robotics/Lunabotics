const std = @import("std");
const linux = std.os.linux;

const can = @import("can.zig");
const VescBus = can.VescBus;
const VescBusConfig = can.VescBusConfig;
const VescConfig = can.VescConfig;

pub fn main(init: std.process.Init) !void {
    const io = init.io;
    const gpa = init.gpa;

    const vesc_configs = [_]VescConfig{
        .{ .vesc_id = 1 },
        .{ .vesc_id = 2 },
    };
    const vesc_bus_config = VescBusConfig{
        .interface_name = "can0",
        .vesc_configs = &vesc_configs,
    };

    var vesc_bus = try VescBus.open(io, gpa, vesc_bus_config, null);
    defer vesc_bus.close();

    std.debug.print("can socket is open now\n", .{});
    while (true) {
        const event = try vesc_bus.readNextEvent(io);
        std.debug.print("event: {any}\n", .{event});
    }
}
