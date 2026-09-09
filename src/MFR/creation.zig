const std = @import("std");
const Io = std.Io;
const meta = std.meta;

pub const NodeConfig = struct {
    node_type: type,
    name: [:0]const u8,
    outputs_to: []const [:0]const u8 = &.{},
};

pub fn CreateOutputBufferType(comptime node_arr: []const NodeConfig) type {
    var field_types: [node_arr.len]type = undefined;
    var names: [node_arr.len][:0]const u8 = undefined;
    inline for (node_arr, 0..) |node, i| {
        if (!@hasDecl(node.node_type, "outputType")) {
            @compileError(std.fmt.comptimePrint("Node {s} does not have an outputType!", .{node.name}));
        }
        field_types[i] = node.node_type.outputType;
        names[i] = node.name;
    }
    const attribute_arr: [names.len]std.builtin.Type.StructField.Attributes = @splat(.{});
    return @Struct(.auto, null, &names, &field_types, &attribute_arr);
}

pub fn CreateInputBufferType(comptime node_arr: []const NodeConfig) type {
    var field_names: [node_arr.len][:0]const u8 = undefined;
    var field_types: [node_arr.len]type = undefined;
    inline for (node_arr, 0..) |node, i| {
        if (!@hasDecl(node.node_type, "inputType")) {
            @compileError(std.fmt.comptimePrint("Node {s} does not have an inputType!", .{node.name}));
        }
        field_names[i] = node.name;
        field_types[i] = node.node_type.inputType;
    }
    return @Struct(.auto, null, &field_names, &field_types, &@splat(.{}));
}

pub fn CreateNodesType(comptime node_arr: []const NodeConfig) type {
    var field_names: [node_arr.len][:0]const u8 = undefined;
    var field_types: [node_arr.len]type = undefined;
    inline for (node_arr, 0..) |node, i| {
        field_names[i] = node.name;
        field_types[i] = node.node_type;
    }
    return @Struct(.auto, null, &field_names, &field_types, &@splat(.{}));
}

pub fn createExecutionFunction(nodesType: type, inputsType: type, outputsType: type, exec_order: []const NodeConfig) fn (*nodesType, *inputsType, *outputsType) void {
    return struct {
        pub fn exec(nodes: *nodesType, inputs: *inputsType, outputs: *outputsType) void {
            inline for (exec_order) |exec_node| {
                var to_exec = &@field(nodes, exec_node.name);
                const input = &@field(inputs, exec_node.name);
                const output = &@field(outputs, exec_node.name);
                to_exec.update(input, output);
            }
        }
    }.exec;
}

pub fn createAsyncGroup(nodesType: type, inputsType: type, outputsType: type, io: Io, comptime async_nodes: []const NodeConfig, nodes: *nodesType, inputs: *inputsType, outputs: *outputsType) Io.Group {
    var group = Io.Group.init;
    inline for (async_nodes) |node| {
        const to_exec = &@field(nodes, node.name);
        const input = &@field(inputs, node.name);
        const output = &@field(outputs, node.name);
        const forever_func = makeForeverFunc(node.node_type);
        group.concurrent(io, forever_func, .{ to_exec, input, output, io }) catch @panic("Failed to setup async group, concurrency not available!");
    }
    return group;
}

pub fn makeForeverFunc(node_type: type) fn (*node_type, *node_type.inputType, *node_type.outputType, Io) void {
    return struct {
        pub fn forever(self: *node_type, input: *node_type.inputType, output: *node_type.outputType, io: Io) void {
            while (true) {
                self.update(input, output);
                io.checkCancel() catch return;
            }
        }
    }.forever;
}

/// The io parameter is only for init and the nodes should not save it
pub fn initNodes(node_type: type, io: Io) node_type {
    var node_struct: node_type = undefined;
    inline for (@typeInfo(node_type).@"struct".fields) |field| {
        const node = &@field(node_struct, field.name);
        node.* = .init(io);
    }
    return node_struct;
}

// This function iterats through all nodes defined in node_arr calling each of their respective outputType linkBuffer function
// The order of the nodes defined in 'outputs_to' in the NodeConfig struct must match the order of the linkBuffer's signiture
pub fn linkOutputs(inputsType: type, outputsType: type, comptime node_arr: []const NodeConfig, inputs: *inputsType, outputs: *outputsType) void {
    inline for (node_arr) |conf| {
        const node_output = &@field(outputs, conf.name);
        // 1 is added to account for self arg in linkBuffer
        const arg_types: [conf.outputs_to.len + 1]type = comptime blk: {
            var tmp: [conf.outputs_to.len + 1]type = undefined;
            tmp[0] = @TypeOf(node_output);
            for (conf.outputs_to, 1..) |dep, i| {
                for (@typeInfo(inputsType).@"struct".fields) |field| {
                    if (std.mem.eql(u8, dep, field.name)) {
                        tmp[i] = *field.type;
                        break;
                    }
                }
            }
            break :blk tmp;
        };

        const arg_type = @Tuple(&arg_types);
        var args: arg_type = undefined;
        args[0] = node_output;
        inline for (conf.outputs_to, 1..) |dep, i| {
            args[i] = &@field(inputs, dep);
        }

        @call(.auto, @TypeOf(node_output.*).linkBuffer, args);
    }
}
