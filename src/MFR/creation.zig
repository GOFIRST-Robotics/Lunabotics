const std = @import("std");
const Io = std.Io;
const meta = std.meta;

pub const OutputBufferingMethod = enum {
    simple,
    double_buffered,
};

pub fn CreateDoubleBuffered(T: type) type {
    return struct {
        mutex: Io.Mutex,
        producer: T,
        consumer: T,

        pub fn init(self: *@This()) void {
            self.mutex = .init;
            self.producer = .{};
            self.consumer = .{};
        }

        pub fn getInputPtr(self: *@This()) *T {
            return &self.consumer;
        }

        pub fn getOutputPtr(self: *@This()) *T {
            return &self.producer;
        }

        pub fn preProducerUpdate(
            self: *@This(),
            _: Io,
        ) Io.Cancelable!void {
            _ = self;
        }

        /// attempt to lock the mutex for swapping
        pub fn postProducerUpdate(
            self: *@This(),
            io: Io,
        ) Io.Cancelable!void {
            try self.mutex.lock(io);
            self.consumer = self.producer;
            self.mutex.unlock(io);
        }

        pub fn preConsumerUpdate(
            self: *@This(),
            io: Io,
        ) Io.Cancelable!void {
            try self.mutex.lock(io);
        }

        pub fn postConsumerUpdate(
            self: *@This(),
            io: Io,
        ) Io.Cancelable!void {
            self.mutex.unlock(io);
        }
    };
}

fn runAllMethodsOnTransfer(
    node_type: type,
    transfer: *CreateNodeTransferBuffer(node_type),
    comptime method_name: []const u8,
    io: Io,
) Io.Cancelable!void {
    inline for (@typeInfo(@TypeOf(transfer.*)).@"struct".fields) |transfer_field| {
        const output_node = &@field(transfer, transfer_field.name);
        inline for (@typeInfo(@TypeOf(output_node.*)).@"struct".fields) |output_node_field| {
            const trans_buf = &@field(output_node, output_node_field.name);
            if (meta.hasMethod(@TypeOf(trans_buf), method_name)) {
                try @call(
                    .auto,
                    @field(@TypeOf(trans_buf.*), method_name),
                    .{ trans_buf, io },
                );
            }
        }
    }
}

pub fn runUpdateForever(
    comptime node_type: type,
) fn (
    Io,
    *node_type,
    *CreateNodeTransferBuffer(node_type),
    *node_type.inputType,
    *node_type.outputType,
) void {
    return struct {
        pub fn forever(
            io: Io,
            node: *node_type,
            transfer: *CreateNodeTransferBuffer(node_type),
            input: *node_type.inputType,
            output: *node_type.outputType,
        ) void {
            while (true) {
                _ = io.swapCancelProtection(.unblocked);
                io.checkCancel() catch return;
                _ = io.swapCancelProtection(.blocked);

                runAllMethodsOnTransfer(
                    node_type,
                    transfer,
                    "preConsumerUpdate",
                    io,
                ) catch unreachable;
                runAllMethodsOnTransfer(
                    node_type,
                    transfer,
                    "preProducerUpdate",
                    io,
                ) catch unreachable;

                @call(
                    .auto,
                    node_type.update,
                    .{ node, input, output },
                );

                runAllMethodsOnTransfer(
                    node_type,
                    transfer,
                    "postConsumerUpdate",
                    io,
                ) catch unreachable;
                runAllMethodsOnTransfer(
                    node_type,
                    transfer,
                    "postProducerUpdate",
                    io,
                ) catch unreachable;
            }
        }
    }.forever;
}

pub fn runPeriodically(
    comptime nodes_type: []const type,
    comptime NodesType: type,
    comptime TransferType: type,
    comptime InputsType: type,
    comptime OutputsType: type,
) fn (
    Io,
    Io.Duration,
    *NodesType,
    *TransferType,
    *InputsType,
    *OutputsType,
) void {
    return struct {
        pub fn runPeriodically(
            io: Io,
            update_speed: Io.Duration,
            nodes: *NodesType,
            transfers: *TransferType,
            inputs: *InputsType,
            outputs: *OutputsType,
        ) void {
            var next_tick = Io.Timestamp.now(io, .awake);

            while (true) {
                _ = io.swapCancelProtection(.unblocked);
                io.checkCancel() catch return;
                _ = io.swapCancelProtection(.blocked);

                inline for (nodes_type) |node_type| {
                    const name = node_type.NodeName;
                    const node = &@field(nodes, name);
                    const transfer = &@field(transfers, name);
                    const input = &@field(inputs, name);
                    const output = &@field(outputs, name);

                    runAllMethodsOnTransfer(
                        @typeInfo(@TypeOf(node)).pointer.child,
                        transfer,
                        "preConsumerUpdate",
                        io,
                    ) catch unreachable;
                    runAllMethodsOnTransfer(
                        @typeInfo(@TypeOf(node)).pointer.child,
                        transfer,
                        "preProducerUpdate",
                        io,
                    ) catch unreachable;

                    @call(
                        .auto,
                        @TypeOf(node.*).update,
                        .{ node, input, output },
                    );

                    runAllMethodsOnTransfer(
                        @typeInfo(@TypeOf(node)).pointer.child,
                        transfer,
                        "postConsumerUpdate",
                        io,
                    ) catch unreachable;
                    runAllMethodsOnTransfer(
                        @typeInfo(@TypeOf(node)).pointer.child,
                        transfer,
                        "postProducerUpdate",
                        io,
                    ) catch unreachable;
                }

                next_tick = Io.Timestamp.addDuration(next_tick, update_speed);
                const now = Io.Timestamp.now(io, .awake);
                const sleep_time = Io.Timestamp.durationTo(now, next_tick);

                if (sleep_time.nanoseconds > 0) {
                    io.sleep(sleep_time, .awake) catch unreachable;
                } else {
                    std.log.warn(
                        "Synchronous loop overrun by {d}ms",
                        .{@divTrunc(-sleep_time.nanoseconds, std.time.ns_per_ms)},
                    );
                    // prevent the loop from trying to catch up so reset expectations
                    next_tick = now;
                }
            }
        }
    }.runPeriodically;
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

fn CreateNodeTransferBuffer(node: type) type {
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
                const child_type = @typeInfo(
                    @FieldType(
                        output_conf.output_node.inputType,
                        @tagName(lit.field),
                    ),
                ).pointer.child;
                literal_types[lit_index] = switch (lit.buffering_type) {
                    .simple => child_type,
                    .double_buffered => CreateDoubleBuffered(child_type),
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
    return @Struct(
        .auto,
        null,
        &output_field_names,
        &output_field_types,
        &@splat(.{}),
    );
}

pub fn CreateTransferBuffer(nodes: []const type) type {
    // names of the nodes providing the outputs
    var field_names: [nodes.len][]const u8 = undefined;
    // type should be a struct of structs where each child struct is
    // the name of the node it outputs to,
    // the fields in the struct will be the output types
    var field_types: [nodes.len]type = undefined;

    inline for (nodes, 0..) |node, node_index| {
        field_names[node_index] = node.NodeName;
        field_types[node_index] = CreateNodeTransferBuffer(node);
    }

    return @Struct(
        .auto,
        null,
        &field_names,
        &field_types,
        &@splat(.{}),
    );
}

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
                    output_field.* = trans_output.getOutputPtr();
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
