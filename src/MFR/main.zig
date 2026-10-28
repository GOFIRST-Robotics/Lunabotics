const std = @import("std");
const linux = std.os.linux;
const Io = std.Io;

const MFR = @import("MFR");
const TeleopStateMachineNode = MFR.TeleopStateMachineNode;
const MechanismNode = MFR.MechanismNode;
const CANOutNode = MFR.CANOutNode;
const CANInNode = MFR.CANInNode;
const ServerNode = MFR.ServerNode;
const creation = @import("creation.zig");
const config = MFR.config;
const on_jetson = config.on_jetson;
const buffering = MFR.buffering_setup;

const logger = std.log.scoped(.main);

const RobotNodes: []const type = &.{
    TeleopStateMachineNode,
    ServerNode,
};

const NodesType = buffering.CreateNodesType(RobotNodes);

const TransferBuffer: type = buffering.CreateTransferBuffer(RobotNodes);
const InputsType: type = buffering.CreateInputs(RobotNodes);
const OutputsType: type = buffering.CreateOutputs(RobotNodes);

// No other file should import this
// This should be an explicit parameter
var kill_robot: std.atomic.Value(bool) = .init(false);

fn interruptHandler(signal: linux.SIG) callconv(.c) void {
    if (signal == .INT) {
        kill_robot.store(true, .seq_cst);
        logger.info("Received Ctrl+C, stopping robot...\n", .{});
    } else {
        std.debug.panic("Got unexpected signal: {t}", .{signal});
    }
}

fn printMetaInfo(T: type) void {
    const info = @typeInfo(T).@"struct";
    inline for (info.fields) |field| {
        @compileLog(field.name ++ ": " ++ @typeName(field.type));
        if (@typeInfo(field.type) == .@"struct") {
            printMetaInfo(field.type);
        }
    }
}

pub fn main(init: std.process.Init) !void {
    logger.info("On jetson: {}\n", .{on_jetson});

    // setup handling signal interrupt
    {
        const sigaction_rc = linux.errno(
            linux.sigaction(
                .INT,
                &.{
                    .handler = .{ .handler = interruptHandler },
                    .mask = linux.sigemptyset(),
                    .flags = 0,
                },
                null,
            ),
        );

        if (sigaction_rc != .SUCCESS) {
            std.debug.panic("Failed to set sigaction ; errno: {t}\n", .{sigaction_rc});
        }
    }

    var transfer: TransferBuffer = undefined;
    buffering.initTransferBuffer(&transfer);
    var inputs: InputsType = undefined;
    var outputs: OutputsType = undefined;
    buffering.tieToTransferBuffer(&transfer, &outputs, &inputs);

    // create nodes
    var nodes: NodesType = undefined;
    nodes.ServerNode.init(init.io);
    nodes.TeleopStateMachine.init();

    var robot_threads: Io.Threaded = .init(init.gpa, .{});
    const robot_io = robot_threads.io();

    var async_group: Io.Group = .init;
    try async_group.concurrent(
        robot_io,
        buffering.runUpdateForever(ServerNode),
        .{
            robot_io,
            &nodes.ServerNode,
            &inputs.ServerNode,
            &outputs.ServerNode,
        },
    );
    defer async_group.cancel(robot_io);

    while (!kill_robot.load(.seq_cst)) {}
}

fn printStruct(out: *std.Io.Writer, value: anytype) !void {
    const T = @TypeOf(value);
    const info = @typeInfo(T).@"struct";
    try out.print("{s} {{\n", .{@typeName(T)});
    // Names and types are parallel arrays, with a guaranteed equal length.
    // (Before, they were one `fields` array of structs. Zig removed that shape.)
    // inline for (info.field_names, info.field_types) |name, Field| {
    inline for (info.fields) |field| {
        const name = field.name;
        const Field = field.type;
        const v = @field(value, name);
        // Strings need {s}. All other types get a usable default from {any}.
        // The branch is comptime. Thus the compiler compiles only one side for
        // each field.
        if (comptime Field == []const u8) {
            try out.print("  {s}: \"{s}\"\n", .{ name, v });
        } else if (@typeInfo(Field) == .pointer) {
            try out.print("  {s}: \"{*}\"\n", .{ name, v });
        } else {
            try out.print("  {s}: {any}\n", .{ name, v });
        }
    }
    try out.print("}}\n", .{});
}
