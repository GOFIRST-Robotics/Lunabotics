const std = @import("std");
const Io = std.Io;

const ServerNode = @import("../controller_com/ServerNode.zig");
pub const inputType = ServerNode.outputType;

pub const outputType = struct {
    drivetrain: DrivetrainState = .{},
    dumper: DumperState = .Disable,
    auger_tilt: AugerTiltState = .Disable,
    auger_push: AugerPushState = .Disable,
    auger_spin: AugerSpinState = .Disable,
};

const DrivetrainState = struct {
    x: f32 = 0, // [-1.0, 1.0]
    y: f32 = 0, // [-1.0, 1.0]
};
const DumperState = enum { Extend, Retract, Disable };
const AugerTiltState = enum { Extend, Retract, Disable };
const AugerPushState = enum { Extend, Retract, Disable };
const AugerSpinState = enum { On, Disable };

const State = enum {
    Driving,
    Digging,
    Dumping,
    Auto,
    Disabled,
};
const SubState = union(enum) {
    drivetrain: DrivetrainState,
    dumper: DumperState,
    auger_tilt: AugerTiltState,
    auger_push: AugerPushState,
    auger_spin: AugerSpinState,
};
// --------- Instance State ---------
var current_state: State = .Driving;
var current_sub_state: SubState = .{ .drivetrain = .{} };

pub fn init(_: Io) @This() {
    return .{};
}

pub fn deinit(self: *@This()) void {
    _ = self;
}

pub fn update(self: *@This(), input: *inputType, output: *outputType) void {
    _ = self;
    const controller = input.*.controller_data;

    switch (current_state) {
        .Driving => driving(controller.right_stick.x, controller.right_stick.y, output),
        .Digging => digging(output),
        .Dumping => dumping(output),
        .Disabled => disabled(output),
        else => {},
    }
}

fn driving(x: f32, y: f32, output: *outputType) void {
    output.drivetrain = .{ x, y };
}

fn dumping(output: *outputType) void {
    if (current_sub_state.dumper == .Extend) {
        output.dumper = .Extend;
        // If sensor sub_state.dumper = .Retract
    } else if (current_sub_state.dumper == .Retract) {
        output.dumper = .Retract;
        // If sensor sub_state.dumper = .Disable
    } else {
        output.dumper = .Disable;
        current_state = .Driving;
    }
}
fn digging(output: *outputType) void {
    _ = output;
}

fn disabled(output: *outputType) void {
    output.drivetrain = .{ 0, 0 };
    output.dumper = .Disable;
    output.auger_tilt = .Disable;
    output.auger_push = .Disable;
    output.auger_spin = .Disable;
}
