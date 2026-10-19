const std = @import("std");
pub const vesc_can = @import("vesc_can.zig");
pub const vesc_datatypes = @import("vesc_datatypes.zig");
pub const vesc_buffer = @import("vesc_buffer.zig");
pub const vesc_packets = @import("vesc_packets.zig");
//pub const simulate_motor = @import("simulate_motor.zig");
pub const can = @import("can.h");

test "test everything" {
    std.testing.refAllDecls(@This());
}
