const std = @import("std");
const Io = std.Io;
const meta = std.meta;
const builtin = @import("builtin");

pub const OutputBufferingMethod = enum {
    simple,
    double_buffered,
};

pub fn CreateDoubleBuffered(T: type) type {
    return struct {
        mutex: Io.Mutex,
        producing: *T,
        consuming: *T,
        buff1: T,
        buff2: T,

        pub fn init(self: *@This()) void {
            self.mutex = .init;
            self.producing = &self.buff1;
            self.consuming = &self.buff2;
        }

        pub fn getInputPtr(self: *const @This()) *T {
            return self.producing;
        }

        pub fn getOutputPtr(self: *const @This()) *T {
            return self.consuming;
        }

        pub fn preConsumerUpdate(
            self: *@This(),
            _: Io,
            _: **@This(),
            _: **@This(),
        ) Io.Cancelable!void {
            _ = self;
        }

        /// attempt to lock the mutex for swapping
        pub fn postConsumerUpdate(
            self: *@This(),
            io: Io,
            input_ptr_to_this: **@This(),
            output_ptr_to_this: **@This(),
        ) Io.Cancelable!void {
            try self.mutex.lock(io);
            std.mem.swap(*T, &self.producing, &self.consuming);
            input_ptr_to_this.* = self.producing;
            output_ptr_to_this.* = self.producing;
            self.mutex.unlock(io);
        }

        pub fn preProducerUpdate(
            self: *@This(),
            io: Io,
            _: **@This(),
            _: **@This(),
        ) Io.Cancelable!void {
            try self.mutex.lock(io);
        }

        pub fn postProducerUpdate(
            self: *@This(),
            io: Io,
            _: **@This(),
            _: **@This(),
        ) Io.Cancelable!void {
            self.mutex.unlock(io);
        }
    };
}

pub fn runUpdateForever(
    comptime node_type: type,
) fn (
    Io,
    *node_type,
    *node_type.inputType,
    *node_type.outputType,
) void {
    return struct {
        pub fn forever(
            io: Io,
            node: *node_type,
            input: *node_type.inputType,
            output: *node_type.outputType,
        ) void {
            while (true) {
                _ = io.swapCancelProtection(.unblocked);
                io.checkCancel() catch return;
                _ = io.swapCancelProtection(.blocked);

                if (meta.hasMethod(@TypeOf(input), "preConsumerUpdate")) {
                    input.preConsumerUpdate();
                }
                if (meta.hasMethod(@TypeOf(output), "preProducerUpdate")) {
                    output.preProducerUpdate();
                }

                @call(
                    .auto,
                    node_type.update,
                    .{ node, input, output },
                );

                if (meta.hasMethod(@TypeOf(input), "postConsumerUpdate")) {
                    input.postConsumerUpdate();
                }
                if (meta.hasMethod(@TypeOf(output), "postProducerUpdate")) {
                    output.postProducerUpdate();
                }
            }
        }
    }.forever;
}

pub const OutputConfig = struct {
    output_node: type,
    output_field: []const struct {
        field: @EnumLiteral(),
        buffering_type: OutputBufferingMethod,
    },
};

pub fn CreateOutputType(comptime configs: []const OutputConfig) type {
    const len = configs.len;
    var field_names: [len][]const u8 = undefined;
    var field_types: [len]type = undefined;

    inline for (configs, 0..) |conf, i| {
        field_names[i] = conf.output_node.NodeName;
        field_types[i] = blk: {
            var lit_names: [conf.output_field.len][]const u8 = undefined;
            var lit_types: [conf.output_field.len]type = undefined;
            inline for (conf.output_field, 0..) |lit, lit_index| {
                lit_names[lit_index] = @tagName(lit.field);
                lit_types[lit_index] = @FieldType(
                    conf.output_node.inputType,
                    @tagName(conf.output_field[lit_index].field),
                );
            }
            break :blk @Struct(
                .auto,
                null,
                &lit_names,
                &lit_types,
                &@splat(.{}),
            );
        };
    }

    return @Struct(
        .auto,
        null,
        &field_names,
        &field_types,
        &@splat(.{}),
    );
}

/// --------    --------
/// |Node A| -> |Node B|
/// --------    --------
///
/// TransferType{
///     NodeA{
///         NodeB{
///             var1,
///             var2,
///         }
///     }
/// }
pub fn CreateTransferBuffer(nodes: []const type) type {
    // names of the nodes providing the outputs
    var field_names: [nodes.len][]const u8 = undefined;
    // type should be a struct of structs where each child struct is
    // the name of the node it outputs to,
    // the fields in the struct will be the output types
    var field_types: [nodes.len]type = undefined;

    inline for (nodes, 0..) |node, node_index| {
        field_names[node_index] = node.NodeName;
        field_types[node_index] = get_node_outputs: {
            const output_configs: []const OutputConfig = &node.outputConfigs;
            // names of the nodes this node outputs to
            var output_field_names: [output_configs.len][]const u8 = undefined;
            // struct types that hold what the outputs are
            var output_field_types: [output_configs.len]type = undefined;

            inline for (output_configs, 0..) |output_conf, output_conf_index| {
                // names of output nodes
                output_field_names[output_conf_index] = output_conf.output_node.NodeName;
                // types of what is being outputted
                output_field_types[output_conf_index] = from_literals: {
                    var literal_names: [output_conf.output_field.len][]const u8 = undefined;
                    var literal_types: [output_conf.output_field.len]type = undefined;

                    inline for (output_conf.output_field, 0..) |lit, lit_index| {
                        literal_names[lit_index] = @tagName(lit.field);
                        const simple_type = meta.Child(
                            @FieldType(
                                output_conf.output_node.inputType,
                                @tagName(lit.field),
                            ),
                        );
                        literal_types[lit_index] = switch (lit.buffering_type) {
                            .simple => simple_type,
                            .double_buffered => CreateDoubleBuffered(simple_type),
                        };
                    }

                    break :from_literals @Struct(
                        .auto,
                        null,
                        &literal_names,
                        &literal_types,
                        &@splat(.{}),
                    );
                };
            }
            break :get_node_outputs @Struct(
                .auto,
                null,
                &output_field_names,
                &output_field_types,
                &@splat(.{}),
            );
        };
    }

    return @Struct(
        .auto,
        null,
        &field_names,
        &field_types,
        &@splat(.{}),
    );
}

// pub fn CreateTransferBuffer(nodes: []const type) type {
//     var field_names: [nodes.len][]const u8 = undefined;
//     var field_types: [nodes.len]type = undefined;
//
//     inline for (nodes, 0..) |node, node_index| {
//         field_names[node_index] = @typeName(node);
//
//         field_types[node_index] = blk: {
//             const outputConfigs: []const OutputConfig = &node.outputConfigs;
//             var node_output_names: [outputConfigs.len][]const u8 = undefined;
//             var node_output_types: [outputConfigs.len]type = undefined;
//             inline for (outputConfigs, 0..) |conf, output_configs_index| {
//                 if (!@hasField(std.meta.FieldEnum(conf.output_node.inputType), @tagName(conf.output_field))) {
//                     @compileError(std.fmt.comptimePrint(
//                         \\Node {s}'s inputType has no field named '{s}'
//                         \\Double check {s}'s outputConfig!
//                     ,
//                         .{ conf.output_node, @tagName(conf.output_field), @typeName(node) },
//                     ));
//                 }
//                 const child_type: type = meta.Child(
//                     @FieldType(
//                         conf.output_node.inputType,
//                         @tagName(conf.output_field),
//                     ),
//                 );
//                 node_output_names[output_configs_index] = @tagName(conf.output_field);
//                 node_output_types[output_configs_index] = switch (conf.buffering_type) {
//                     .simple => child_type,
//                     .double_buffered => CreateDoubleBuffered(child_type),
//                 };
//             }
//             break :blk @Struct(
//                 .auto,
//                 null,
//                 &node_output_names,
//                 &node_output_types,
//                 &@splat(.{}),
//             );
//         };
//     }
//
//     return @Struct(
//         .auto,
//         null,
//         &field_names,
//         &field_types,
//         &@splat(.{}),
//     );
// }

pub fn initTransferBuffer(transfer_instance: anytype) void {
    const transfer_info_ptr = @typeInfo(@TypeOf(transfer_instance)).pointer;
    const transfer_info = @typeInfo(transfer_info_ptr.child).@"struct";

    inline for (transfer_info.fields) |transfer_field| {
        const node = &@field(transfer_instance, transfer_field.name);
        const node_info = @typeInfo(@typeInfo(@TypeOf(node)).pointer.child).@"struct";

        inline for (node_info.fields) |output_node_info| {
            const output_node = &@field(node, output_node_info.name);

            inline for (@typeInfo(output_node_info.type).@"struct".fields) |output_info| {
                var output_var: *output_info.type = &@field(output_node, output_info.name);
                if (std.meta.hasMethod(output_info.type, "init")) {
                    output_var.init();
                } else {
                    output_var.* = .{};
                }
            }
        }
    }
}

pub fn tieToTransferBuffer(transfer: anytype, outputs: anytype, inputs: anytype) void {
    inline for (@typeInfo(@TypeOf(transfer.*)).@"struct".fields) |node_field| {
        const trans_node = &@field(transfer, node_field.name);

        // get node to from outputs field
        const outputs_field = &@field(outputs, node_field.name);

        inline for (@typeInfo(@TypeOf(trans_node.*)).@"struct".fields) |node_output_field| {
            var output = &@field(outputs_field, node_output_field.name);
            var input = &@field(inputs, node_output_field.name);
            const trans_node_output = &@field(trans_node, node_output_field.name);

            inline for (@typeInfo(@TypeOf(output.*)).@"struct".fields) |input_field_info| {
                const output_field: *input_field_info.type = &@field(output, input_field_info.name);
                const input_field: *input_field_info.type = &@field(input, input_field_info.name);

                const trans_output = &@field(trans_node_output, input_field_info.name);

                if (meta.hasMethod(@TypeOf(trans_output), "getInputPtr")) {
                    input_field.* = trans_output.getInputPtr();
                    input_field.* = trans_output.getOutputPtr();
                } else {
                    input_field.* = trans_output;
                    output_field.* = trans_output;
                }
            }
        }
    }
}

pub fn CreateInputs(nodes: []const type) type {
    var field_names: [nodes.len][]const u8 = undefined;
    var field_types: [nodes.len]type = undefined;

    inline for (nodes, 0..) |node, node_index| {
        field_names[node_index] = node.NodeName;
        field_types[node_index] = node.inputType;
        inline for (@typeInfo(node.inputType).@"struct".fields) |field| {
            if (@typeInfo(field.type) != .pointer) {
                @compileError(std.fmt.comptimePrint(
                    "All fields in 'inputType' should be a pointer. Double check {s}'s 'inputType' definition",
                    .{
                        node.name,
                    },
                ));
            }
        }
    }

    return @Struct(
        .auto,
        null,
        &field_names,
        &field_types,
        &@splat(.{}),
    );
}

pub fn CreateOutputs(nodes: []const type) type {
    var field_names: [nodes.len][]const u8 = undefined;
    var field_types: [nodes.len]type = undefined;

    inline for (nodes, 0..) |node, node_index| {
        field_names[node_index] = node.NodeName;
        field_types[node_index] = node.outputType;
        inline for (@typeInfo(node.inputType).@"struct".fields) |field| {
            if (@typeInfo(field.type) != .pointer) {
                @compileError(std.fmt.comptimePrint(
                    "All fields in 'inputType' should be a pointer. Double check {s}'s 'inputType' definition",
                    .{
                        node.name,
                    },
                ));
            }
        }
    }

    return @Struct(
        .auto,
        null,
        &field_names,
        &field_types,
        &@splat(.{}),
    );
}

// Creates a new function will repeat func until shoutdow_flag is true
// pub fn runFuncUntilRobotShutdown(
//     shutdown_flag: *std.atomic.Value(bool),
//     // io: Io,
//     func: anytype,
// ) @TypeOf(func) {
//     const args_type = @typeInfo(std.meta.ArgsTuple(@TypeOf(func))).@"struct".fields;
//     std.debug.assert(args_type.len == 3);
//     return struct {
//         pub fn forever(
//             node: args_type[0].type,
//             input: args_type[1].type,
//             output: args_type[2].type,
//         ) void {
//             while (!shutdown_flag.load(.seq_cst)) {
//                 func(node, input, output);
//             }
//         }
//     }.forever;
// }
//

pub fn CreateNodesType(comptime nodes: []const type) type {
    var field_names: [nodes.len][]const u8 = undefined;
    var field_types: [nodes.len]type = undefined;
    inline for (nodes, 0..) |node, i| {
        field_names[i] = node.NodeName;
        field_types[i] = node;
    }
    return @Struct(
        .auto,
        null,
        &field_names,
        &field_types,
        &@splat(.{}),
    );
}
