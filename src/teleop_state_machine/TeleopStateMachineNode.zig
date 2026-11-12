const std = @import("std");
const Io = std.Io;

const CANInNode = @import("../can_bus/CANInNode.zig");
const protocol = @import("../controller_com/protocol.zig");
const ControllerData = protocol.ControllerData;
const StreamDeckData = protocol.StreamDeckData;
const creation = @import("../MFR/creation.zig");

const TeleopStateMachine = @This();
pub const NodeName: []const u8 = "TeleopStateMachine";

const logger = std.log.scoped(.teleop_state_machine);

const DrivetrainState = struct {
    x: f32 = 0, // [-1.0, 1.0]
    y: f32 = 0, // [-1.0, 1.0]
};
const JOYSTICK_DEADBAND = 0.05;
const TILT_CURRENT_THRESHOLD = 0.01;

const RobotState = union(enum) {
    Driving: void,
    TiltAugerDown: struct { avg_current: f32 },
    BeginDig: void,
    Digging: DiggingState,
    Auto: void,
    Idle: void,
    Disable: void,
};
const DiggingState = enum { Plunging, DiggingBottom };

io: Io,
// If null then the devices are either not plugged in or data is stale
last_controller_state: ?ControllerData,
last_stream_deck_state: ?StreamDeckData,
state: RobotState,
prev_timestamp: Io.Timestamp,

pub const ControllerInfo = struct {
    controller_state: ?ControllerData = null,
    stream_deck_state: ?StreamDeckData = null,
};

pub const inputType = struct {
    controller_info: *ControllerInfo,
    tilt_motor_current: *f32,
};

pub const outputConfigs: [0]creation.OutputConfig = .{};

pub const outputType = struct {};

pub fn init(self: *@This(), io: Io) void {
    self.io = io;
    self.state = .Idle;
    self.last_controller_state = null;
    self.last_stream_deck_state = null;
}

pub fn update(self: *@This(), input: *inputType, output: *outputType) void {
    defer self.last_controller_state = input.controller_info.controller_state;
    defer self.last_stream_deck_state = input.controller_info.stream_deck_state;

    const controller = input.controller_info.controller_state;
    if (controller == null) {
        disable(output); // Meh
        // return;
    }
    switch (self.state) {
        .Driving => driving(controller.right_stick.x, controller.right_stick.y, output),
        .TiltAugerDown => |s| tiltAugerDown(self, &s.avg_current, input.tilt_motor_current, output),
        .BeginDig => beginDig(self, input.auger_speed, input.aux_speed, output),
        .Idle => idle(self, controller, output),
        else => {},
    }
}

fn driving(x: f32, y: f32, output: *outputType) void {
    output.drivetrain = .{ x, y };
}

fn tiltAugerDown(self: *TeleopStateMachine, avg_current: *f32, tilt_current: f32, output: *outputType) void {
    const dt = Io.Timestamp.now(self.io, .awake).nanoseconds - self.prev_timestamp.nanoseconds;
    output.auger_tilt = .Down;
    avg_current.* += (tilt_current * dt);
    if (@abs(avg_current) < TILT_CURRENT_THRESHOLD) {
        self.state = .BeginDig;
    }
}

fn beginDig(self: *TeleopStateMachine, auger_speed: f32, aux_speed: f32, output: *outputType) void {
    const augerThreshold = 1;
    const auxThreshold = 1;
    output.auger_spin = .Forward;
    output.aux_dig = .Enable;
    if (auger_speed > augerThreshold and aux_speed > auxThreshold) {
        self.state = DiggingState.Plunging;
    }
}

fn idle(self: *TeleopStateMachine, controller: ControllerData, output: *outputType) void {
    disable(output);
    if (controller.buttons.north) {
        self.state = .TiltAugerDown;
    } else if (@abs(controller.right_stick.x) > JOYSTICK_DEADBAND or @abs(controller.right_stick.y) > JOYSTICK_DEADBAND) {
        self.state = .Driving;
    }
}

fn disable(output: *outputType) void {
    output.drivetrain = .{ 0, 0 };
    output.dumper = .Disable;
    output.auger_tilt = .Disable;
    output.auger_push = .Disable;
    output.auger_spin = .Disable;
}
