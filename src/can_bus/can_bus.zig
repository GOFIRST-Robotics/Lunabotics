const std = @import("std");
const Io = std.Io;

// CAN ID of the Jetson
pub const controller_id: u8 = 0;

// VESC IDs of each motor on the bus
pub const Motor = enum(u8) {
    // Eventually, the actual motors in the robot should be put here
    TestMotor = 1,

    pub fn byId(motor_id: u8) ?@This() {
        return std.enums.fromInt(@This(), motor_id);
    }

    pub fn id(self: @This()) u8 {
        return @backingInt(self);
    }
};
pub const motors = std.enums.values(Motor);

pub fn PerMotor(inner: type) type {
    return struct {
        values: [motors.len]inner = @splat(.{}),

        pub fn forMotor(self: *@This(), motor: Motor) *inner {
            inline for (motors, 0..) |m, i| {
                if (m == motor) return &self.values[i];
            }
            unreachable;
        }
    };
}
