const std = @import("std");
const linux = std.os.linux;
const Io = std.Io;

const MFR = @import("MFR");
const MechanismNode = MFR.MechanismNode;
const CANOutNode = MFR.CANOutNode;
const CANInNode = MFR.CANInNode;
const ServerNode = MFR.ServerNode;
const creation = @import("creation.zig");
const config = MFR.config;
const on_jetson = config.on_jetson;
const NodeConfig = creation.NodeConfig;

const logger = std.log.scoped(.main);

const zig_vesc_can = @import("zig-vesc-can");
const socket_can = zig_vesc_can.socket_can;

pub const mechanism_config: NodeConfig = .{
    .node_type = MechanismNode,
    .name = "mechanism",
    .outputs_to = &.{canout_config.name},
};
pub const canout_config: NodeConfig = .{
    .node_type = CANOutNode,
    .name = "can_out",
};
pub const canin_config: NodeConfig = .{
    .node_type = CANInNode,
    .name = "can_in",
};
pub const server_config: NodeConfig = .{
    .node_type = ServerNode,
    .name = "server",
    .outputs_to = &.{mechanism_config.name},
};

const node_config = [_]NodeConfig{
    mechanism_config,
    canout_config,
    canin_config,
    server_config,
};

pub const OutputsType = creation.CreateOutputBufferType(&node_config);
pub const InputsType = creation.CreateInputBufferType(&node_config);
pub const NodesType = creation.CreateNodesType(&node_config);

const main_control_exec_order = [_]NodeConfig{
    mechanism_config,
    canout_config,
};

const async_nodes = [_]NodeConfig{
    server_config,
    canin_config,
};

const synchronous_spin = creation.createExecutionFunction(
    NodesType,
    InputsType,
    OutputsType,
    &main_control_exec_order,
);

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

    var inputs: InputsType = undefined;

    var outputs: OutputsType = undefined;

    var robot_threaded: Io.Threaded = .init(init.gpa, .{});
    defer robot_threaded.deinit();
    const robot_io = robot_threaded.io();
    var robot_io_group: Io.Group = .init;

    var nodes: NodesType = creation.initNodes(NodesType, init.io);

    creation.linkOutputs(InputsType, OutputsType, &node_config, &inputs, &outputs);

    // every node in the async group has to have the same io
    inline for (async_nodes) |node| {
        var node_instance = &@field(nodes, node.name);
        node_instance.io = robot_io;
    }

    // Create the looping function for each IO node and
    // add it to the robot_io group
    inline for (async_nodes) |node| {
        const node_type = node.node_type;
        const node_instance = &@field(nodes, node.name);
        const input = &@field(inputs, node.name);
        const output = &@field(outputs, node.name);
        const func: @TypeOf(node_type.update) = creation.runFuncUntilRobotShutdown(
            &kill_robot,
            node_type.update,
        );

        try robot_io_group.concurrent(robot_io, func, .{ node_instance, input, output });
    }

    defer robot_io_group.cancel(robot_io);

    // synchronous tasks will run every 20ms
    const update_speed_ns = Io.Duration.fromMilliseconds(20);
    var next_tick = Io.Timestamp.now(init.io, .awake);

    while (!kill_robot.load(.seq_cst)) {
        synchronous_spin(&nodes, &inputs, &outputs);

        next_tick = Io.Timestamp.addDuration(next_tick, update_speed_ns);
        const now = Io.Timestamp.now(init.io, .awake);
        const sleep_time = Io.Timestamp.durationTo(now, next_tick);

        if (sleep_time.nanoseconds > 0) {
            try init.io.sleep(sleep_time, .awake);
        } else {
            logger.warn(
                "Main synchrounous loop overrun by {d}ms",
                .{@divTrunc(-sleep_time.nanoseconds, std.time.ns_per_ms)},
            );
            // prevent the loop from trying to catch up so reset expectations
            next_tick = now;
        }
    }
}
