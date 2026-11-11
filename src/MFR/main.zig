const std = @import("std");
const linux = std.os.linux;
const Io = std.Io;

const MFR = @import("MFR");
const zig_vesc_can = MFR.zig_vesc_can;
const Foxglove = MFR.Foxglove;

const TeleopStateMachineNode = MFR.TeleopStateMachineNode;
const MechanismNode = MFR.MechanismNode;
const CANOutNode = MFR.CANOutNode;
const CANInNode = MFR.CANInNode;
const ServerNode = MFR.ServerNode;

const config = MFR.config;
const on_jetson = config.on_jetson;
const creation = MFR.creation;

const logger = std.log.scoped(.main);

const RobotNodes: []const type = &.{
    CANInNode,
    TeleopStateMachineNode,
    ServerNode,
    CANOutNode,
};

const NodesType = creation.CreateNodesType(RobotNodes);

const TransferBuffer: type = creation.CreateTransferBuffer(RobotNodes);
const InputsType: type = creation.CreateInputs(RobotNodes);
const OutputsType: type = creation.CreateOutputs(RobotNodes);

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

pub fn main(init: std.process.Init) !void {
    logger.info("On jetson: {}\n", .{on_jetson});
    Foxglove.init(init.gpa);
    defer Foxglove.deinit();

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

    var robot_threads: Io.Threaded = .init(init.gpa, .{});
    const robot_io = robot_threads.io();

    var transfer: TransferBuffer = undefined;
    creation.initTransferBuffer(&transfer);
    var inputs: InputsType = undefined;
    var outputs: OutputsType = undefined;
    creation.tieToTransferBuffer(&transfer, &outputs, &inputs);

    const canbus_name = config.canbus_name;
    const can_socket = zig_vesc_can.can.openSocket(canbus_name, null) catch std.debug.panic("failed to connect to CAN bus '{s}'\n", .{canbus_name.*});
    defer zig_vesc_can.can.closeSocket(can_socket);

    // create nodes
    var nodes: NodesType = undefined;
    nodes.CANInNode.init(robot_io, can_socket);
    nodes.ServerNode.init(robot_io);
    nodes.TeleopStateMachine.init();
    nodes.CANOutNode.init(robot_io, can_socket);

    var async_group: Io.Group = .init;
    try async_group.concurrent(
        robot_io,
        creation.runUpdateForever(CANInNode),
        .{
            robot_io,
            &nodes.CANInNode,
            &transfer.CANInNode,
            &inputs.CANInNode,
            &outputs.CANInNode,
        },
    );
    try async_group.concurrent(
        robot_io,
        creation.runUpdateForever(ServerNode),
        .{
            robot_io,
            &nodes.ServerNode,
            &transfer.ServerNode,
            &inputs.ServerNode,
            &outputs.ServerNode,
        },
    );
    try async_group.concurrent(
        robot_io,
        creation.runPeriodically(
            &.{ TeleopStateMachineNode, CANOutNode },
            NodesType,
            TransferBuffer,
            InputsType,
            OutputsType,
        ),
        .{
            robot_io,
            .fromMilliseconds(20),
            &nodes,
            &transfer,
            &inputs,
            &outputs,
        },
    );
    defer async_group.cancel(robot_io);

    while (!kill_robot.load(.seq_cst)) {}
}
