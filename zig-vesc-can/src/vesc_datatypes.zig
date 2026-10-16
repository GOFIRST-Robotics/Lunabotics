const std = @import("std");
const linux = std.os.linux;
const vesc_packets = @import("vesc_packets.zig");

pub fn WithBCMContext(T: type) type {
    return struct {
        value: T,
        timestamp: linux.kernel_timespec,
        is_alive: bool,
    };
}

pub const MotorStatus = struct {
    status1: ?WithBCMContext(vesc_packets.Status1) = null,
    status2: ?WithBCMContext(vesc_packets.Status2) = null,
    status3: ?WithBCMContext(vesc_packets.Status3) = null,
    status4: ?WithBCMContext(vesc_packets.Status4) = null,
    status5: ?WithBCMContext(vesc_packets.Status5) = null,
    status6: ?WithBCMContext(vesc_packets.Status6) = null,
};
