const std = @import("std");
const Io = std.Io;
const CANInNode = @import("../can_bus/CANInNode.zig");
const vesc_datatypes = @import("../can_bus/vesc_datatypes.zig");
const protocol = @import("../controller_com/protocol.zig");
const ControllerData = protocol.ControllerData;
const StreamDeckData = protocol.StreamDeckData;
const buffering = @import("../MFR/buffering_setup.zig");

const TeleopStateMachine = @This();
pub const NodeName: []const u8 = "TeleopStateMachine";

const logger = std.log.scoped(.teleop_state_machine);

const RobotState = enum {
    disabled, // Robot is doing nothing, this should be a safe state
    driving,
};

state: RobotState,
// If null then the devices are either not plugged in or data is stale
last_controller_state: ?ControllerData,
last_stream_deck_state: ?StreamDeckData,

pub const ControllerInfo = struct {
    controller_state: ?ControllerData = null,
    stream_deck_state: ?StreamDeckData = null,
};

pub const inputType = struct {
    controller_info: *ControllerInfo,
};

pub const outputConfigs = [_]buffering.OutputConfig{};

pub const outputType = struct {};

pub const execution_group: @EnumLiteral() = .logic_chain;

pub fn init(self: *@This()) void {
    self.state = .disabled;
    self.last_controller_state = null;
    self.last_stream_deck_state = null;
}

pub fn update(self: *@This(), input: *inputType, output: *outputType) void {
    _ = output;
    _ = self;
    _ = input;
    // defer self.last_controller_state = input.controller_state;
    // defer self.last_stream_deck_state = input.stream_deck_state;
    //
    // if (input.controller_state) |con_state| {
    //     logger.info("{f}", .{con_state});
    // } else {
    //     logger.info("No controller", .{});
    // }
}
