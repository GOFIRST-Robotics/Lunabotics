const std = @import("std");
pub const can = @import("can.zig");
pub const commands = @import("commands.zig");
pub const packets = @import("packets.zig");
//pub const simulate_motor = @import("simulate_motor.zig");
pub const vesc_buffer = @import("vesc_buffer.zig");

test "test everything" {
    std.testing.refAllDecls(@This());
}
