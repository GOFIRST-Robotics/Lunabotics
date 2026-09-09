const std = @import("std");
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

pub fn main(init: std.process.Init) !void {
    std.debug.print("On jetson: {}\n", .{on_jetson});
    var inputs: InputsType = undefined;

    var outputs: OutputsType = undefined;

    var async_group_threaded: Io.Threaded = .init(init.gpa, .{});
    const async_group_io = async_group_threaded.io();
    defer async_group_threaded.deinit();

    var nodes: NodesType = creation.initNodes(NodesType, init.io);

    creation.linkOutputs(InputsType, OutputsType, &node_config, &inputs, &outputs);

    // every node in the async group has to have the same io
    inline for (async_nodes) |node| {
        var node_instance = &@field(nodes, node.name);
        node_instance.io = async_group_io;
        // std.debug.print("{s} ; {any}\n", .{ node.name, &node_instance.io });
    }

    var async_group = creation.createAsyncGroup(
        NodesType,
        InputsType,
        OutputsType,
        async_group_io,
        &async_nodes,
        &nodes,
        &inputs,
        &outputs,
    );

    // try async_group.await(init.io);

    defer async_group.cancel(init.io);

    while (true) {
        synchronous_spin(&nodes, &inputs, &outputs);
        try init.io.sleep(.fromSeconds(1), .awake);
    }
}
