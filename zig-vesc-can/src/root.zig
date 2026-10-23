const std = @import("std");
pub const can = @import("can.zig");
pub const codec = @import("codec.zig");
pub const commands = @import("commands.zig");
pub const packets = @import("packets.zig");
//pub const simulate_motor = @import("simulate_motor.zig");

test "test everything" {
    std.testing.refAllDecls(@This());
}
