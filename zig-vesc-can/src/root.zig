const std = @import("std");
pub const socket_can = @import("socket_can.zig");
pub const vesc_datatypes = @import("vesc_datatypes.zig");
pub const packet_io = @import("packet_io.zig");
pub const packets_in = @import("packets_in.zig");
pub const packets_out = @import("packets_out.zig");
pub const simulate_motor = @import("simulate_motor.zig");
pub const can = @import("can.h");

test "test everything" {
    std.testing.refAllDecls(@This());
}
