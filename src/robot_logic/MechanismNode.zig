const std = @import("std");
const Io = std.Io;
const CANOutNode = @import("../can_bus/CANOutNode.zig");

const MechanismNode = @This();

const logger = std.log.scoped(.mech);

// defaults are values which stop everything
pub const Instructions = struct {
    extend: ?bool = null,
    spin_rpm: f32 = 0,
    plundge_rpm: f32 = 0,
};
pub const Safty = struct {
    plundge_pos: i64,
    tilt_pos: i64,
    spin_rpm: i64,
};
pub const inputType = struct {
    instructions: Instructions,
    safty: Safty,
};

pub const outputType = struct {
    can_out: *CANOutNode.inputType,
    pub fn linkBuffer(self: *@This(), can_out: *CANOutNode.inputType) void {
        self.can_out = can_out;
    }
};

pub fn init(_: Io) @This() {
    return .{};
}

pub fn deinit(self: *@This()) void {
    _ = self;
}
pub fn update(self: *@This(), input: *inputType, output: *outputType) void {
    _ = &self;
    const is_safe = input.safty.plundge_pos > 10;
    if (is_safe) {
        output.can_out.motor_current = 51;
    } else {
        output.can_out.motor_current = 0.5;
    }
}
