const std = @import("std");
const Io = std.Io;
const linux = std.os.linux;
const zig_vesc_can = @import("zig-vesc-can");
const socket_can = zig_vesc_can.socket_can;
const vesc_datatypes = zig_vesc_can.vesc_datatypes;
const can = @import("can.h");
const json = std.json;
const panic = std.debug.panic;
const Dashboard = @import("Dashboard.zig");

const ReceiveConfig = struct {
    command: vesc_datatypes.CommandType,
    timeout: can.bcm_timeval,
    rate_limit: can.bcm_timeval,
};

const DeferConfigs = struct {
    config_name: []const u8,
    messages: []ReceiveConfig,
};

const VescMonitorConfig = struct {
    name: []const u8,
    can_id: u8,
    defer_configs: [][]const u8,
    // These will overide the ones found in defer_config
    message_configs: []ReceiveConfig,
};

const MonitoringConfig = struct {
    canbus_name: []const u8,
    defer_configs: []DeferConfigs,
    vesc_configs: []VescMonitorConfig,
    layout: [][][]const u8,
};

const instruction_commands = [_]vesc_datatypes.CommandType{
    .SET_DUTY,
    .SET_CURRENT,
    .SET_RPM,
    .SET_POS,
};

const InstructionEnum = make_instruction_enum: {
    var names: [instruction_commands.len][]const u8 = undefined;
    var values: [instruction_commands.len]u8 = undefined;
    for (instruction_commands, 0..) |command, i| {
        names[i] = @tagName(command);
        values[i] = @intFromEnum(command);
    }
    break :make_instruction_enum @Enum(
        u8,
        .exhaustive,
        &names,
        &values,
    );
};

const Command = union(InstructionEnum) {
    SET_DUTY: vesc_datatypes.SetDuty,
    SET_CURRENT: vesc_datatypes.SetCurrent,
    SET_RPM: vesc_datatypes.SetRPM,
    SET_POS: vesc_datatypes.SetPos,
};

pub const MotorInfo = struct {
    name: []const u8,
    status: vesc_datatypes.MotorStatus,
    last_command: struct { command: Command, timestamp: linux.kernel_timespec },
};

pub const MotorMap = std.AutoHashMap(u8, MotorInfo);

pub fn main(init: std.process.Init) !void {
    const io = init.io;
    const arena = init.arena;
    const gpa = init.gpa;

    if (init.minimal.args.vector.len != 2) {
        panic("Give the path to the config file to use\n", .{});
    }

    // Open and parse json from provided config

    const monitor_config: MonitoringConfig = get_config: {
        var config_file = Io.Dir.openFile(
            Io.Dir.cwd(),
            io,
            std.mem.span(init.minimal.args.vector[1]),
            .{
                .allow_directory = false,
                .mode = .read_only,
            },
        ) catch |err| {
            var current_path: [1024 * 2]u8 = undefined;
            const path_len = try Io.Dir.cwd().realPath(io, &current_path);
            panic(
                "Failed to open file '{s}/{s}'\n'; {t}",
                .{
                    current_path[0..path_len],
                    init.minimal.args.vector[1],
                    err,
                },
            );
        };
        defer config_file.close(io);

        var file_read_buff: [1024 * 4]u8 = @splat(0);
        var file_reader = config_file.reader(io, &file_read_buff);

        const json_str = try file_reader.interface.allocRemaining(gpa, .unlimited);
        defer gpa.free(json_str);

        var json_scanner = json.Scanner.initStreaming(gpa);
        defer json_scanner.deinit();

        var diagnostics: json.Diagnostics = .{};
        json_scanner.enableDiagnostics(&diagnostics);
        json_scanner.feedInput(json_str);
        json_scanner.endInput();
        break :get_config json.parseFromTokenSourceLeaky(
            MonitoringConfig,
            arena.allocator(),
            &json_scanner,
            .{ .allocate = .alloc_always },
        ) catch |err| {
            panic(
                \\Failed to parse json in config file
                \\{t} found out line {d} and column {d} 
            ,
                .{
                    err,
                    diagnostics.getLine(),
                    diagnostics.getColumn(),
                },
            );
        };
    };

    std.debug.print("\nconfig: {}\n", .{monitor_config});

    var open_can_errno: linux.E = .SUCCESS;
    const can_socket: linux.socket_t = socket_can.openCANBCM(monitor_config.canbus_name, &open_can_errno) catch |err| {
        panic(
            \\Failed to open "{s}"
            \\{t} ; {t}
        ,
            .{
                monitor_config.canbus_name,
                err,
                open_can_errno,
            },
        );
    };
    defer _ = linux.close(can_socket);

    // Configure every motor with CAN BCM

    // First configure every instruction command
    for (monitor_config.vesc_configs) |conf| {
        for (instruction_commands) |instruction| {
            var errno: linux.E = .SUCCESS;
            socket_can.BCMConfigureIdReceive(
                can_socket,
                .{
                    .rate_limit = .{
                        .tv_sec = 0,
                        .tv_usec = 0,
                    },
                    .timeout = .{
                        .tv_sec = 0,
                        .tv_usec = 0,
                    },
                    .filter_id = .{
                        .vesc_id = conf.can_id,
                        .command_type = instruction,
                    },
                    .recveive_all = true,
                },
                &errno,
            ) catch {
                panic("Failed to configure BCM for vesc config {}; errno: {t}\n", .{ conf, errno });
            };
        }
    }

    var vesc_map: MotorMap = .init(gpa);
    defer vesc_map.deinit();
    try vesc_map.ensureTotalCapacity(@intCast(monitor_config.vesc_configs.len));

    {
        var defer_configs_map: std.StringHashMap([]ReceiveConfig) = .init(gpa);
        defer defer_configs_map.deinit();
        try defer_configs_map.ensureTotalCapacity(@intCast(monitor_config.defer_configs.len));

        for (monitor_config.defer_configs) |conf| {
            defer_configs_map.putAssumeCapacity(conf.config_name, conf.messages);
        }

        var bcm_configs: std.AutoHashMap(socket_can.Id, ReceiveConfig) = .init(gpa);
        defer bcm_configs.deinit();
        try bcm_configs.ensureTotalCapacity(100);

        var current_time: linux.timespec = undefined;
        const clock_errno = linux.errno(linux.clock_gettime(.REALTIME, &current_time));
        if (clock_errno != .SUCCESS) {
            std.debug.panic("Failed to get time with clock gettime, Errno: {t}", .{clock_errno});
        }

        for (monitor_config.vesc_configs) |vesc_conf| {
            try vesc_map.put(vesc_conf.can_id, .{
                .name = vesc_conf.name,
                .last_command = .{
                    .command = .{ .SET_DUTY = .{ .duty = 0 } },
                    .timestamp = .{ .nsec = current_time.nsec, .sec = current_time.sec },
                },
                .status = .{},
            });

            for (vesc_conf.defer_configs) |dconf| {
                const recv_configs = defer_configs_map.getEntry(dconf) orelse panic(
                    "Failed to find defer config \"{s}\" for motor \"{s}\"",
                    .{ dconf, vesc_conf.name },
                );
                for (recv_configs.value_ptr.*) |conf| {
                    const bcm_conf = try bcm_configs.getOrPut(socket_can.Id{
                        .command_type = conf.command,
                        .vesc_id = vesc_conf.can_id,
                    });

                    bcm_conf.value_ptr.* = conf;
                }
            }

            for (vesc_conf.message_configs) |conf| {
                const bcm_conf = try bcm_configs.getOrPut(socket_can.Id{
                    .command_type = conf.command,
                    .vesc_id = vesc_conf.can_id,
                });

                bcm_conf.value_ptr.* = conf;
            }
        }

        var bcm_configs_iter = bcm_configs.iterator();
        while (bcm_configs_iter.next()) |c| {
            const id = c.key_ptr.*;
            const conf = c.value_ptr.*;

            // if it is a command then every frame is wanted
            const is_command = std.mem.containsAtLeast(
                vesc_datatypes.CommandType,
                &instruction_commands,
                1,
                &.{id.command_type},
            );

            var config_errno: linux.E = .SUCCESS;
            socket_can.BCMConfigureIdReceive(
                can_socket,
                .{
                    .filter_id = id,
                    .recveive_all = is_command,
                    .timeout = conf.timeout,
                    .rate_limit = conf.rate_limit,
                },
                &config_errno,
            ) catch {
                panic("Failed to configure with id {} and params {} ; errno {t}\n", .{ id, conf, config_errno });
            };
        }
    }

    // Main loop that will give updates until program is terminated

    var update_alive: bool = true;

    var map_update = try io.concurrent(updateMapInfo, .{
        io,
        can_socket,
        &vesc_map,
        &update_alive,
    });

    const layout = try convertLayout(gpa, monitor_config);

    var dashboard: Dashboard = undefined;
    dashboard.init(io, layout);

    while (update_alive) {
        try io.sleep(.fromMilliseconds(50), .awake);
        try dashboard.draw(&vesc_map);
    }

    try map_update.cancel(io);
}

fn updateMapInfo(io: Io, can_socket: linux.socket_t, map: *MotorMap, update_alive: *bool) !void {
    errdefer update_alive.* = false;
    while (true) {
        std.debug.print("running", .{});
        const frame, const timestamp = socket_can.recvmesgBCM2(
            io,
            can_socket,
            1,
        ) catch |err| return switch (err) {
            error.Canceled => {},
            else => err,
        };

        const id: socket_can.Id = @bitCast(frame.msg_head.can_id);
        const map_data = map.getPtr(id.vesc_id) orelse unreachable;
        std.debug.print("got update {}\n", .{id});
        switch (frame.msg_head.opcode) {
            can.RX_CHANGED => {
                switch (id.command_type) {
                    .SET_DUTY => {
                        map_data.last_command.command = .{ .SET_DUTY = @bitCast(frame.can_frames[0].data) };
                        map_data.last_command.timestamp = timestamp;
                    },
                    .SET_CURRENT => {
                        map_data.last_command.command = .{ .SET_CURRENT = @bitCast(frame.can_frames[0].data) };
                        map_data.last_command.timestamp = timestamp;
                    },
                    .SET_RPM => {
                        map_data.last_command.command = .{ .SET_RPM = @bitCast(frame.can_frames[0].data) };
                        map_data.last_command.timestamp = timestamp;
                    },
                    .SET_POS => {
                        map_data.last_command.command = .{ .SET_POS = @bitCast(frame.can_frames[0].data) };
                        map_data.last_command.timestamp = timestamp;
                    },
                    .STATUS => map_data.status.status1 = .{
                        .timestamp = timestamp,
                        .is_alive = true,
                        .value = .parse(frame.can_frames[0]),
                    },
                    .STATUS_2 => map_data.status.status2 = .{
                        .timestamp = timestamp,
                        .is_alive = true,
                        .value = .parse(frame.can_frames[0]),
                    },
                    .STATUS_5 => map_data.status.status5 = .{
                        .timestamp = timestamp,
                        .is_alive = true,
                        .value = .parse(frame.can_frames[0]),
                    },
                    else => panic("Got unexpected case: {t}\n", .{id.command_type}),
                }
            },
            can.RX_TIMEOUT => {
                switch (id.command_type) {
                    .STATUS => map_data.status.status1 = .{
                        .is_alive = false,
                        .value = undefined,
                        .timestamp = timestamp,
                    },
                    .STATUS_2 => map_data.status.status2 = .{
                        .is_alive = false,
                        .value = undefined,
                        .timestamp = timestamp,
                    },
                    .STATUS_5 => map_data.status.status5 = .{
                        .is_alive = false,
                        .value = undefined,
                        .timestamp = timestamp,
                    },
                    else => {
                        panic("Got unexpected case: {t}\n", .{id.command_type});
                    },
                }
            },
            else => panic("Got unexpected opcode: {d}\n", .{frame.msg_head.opcode}),
        }
    }
}

/// Turn the layout defined by motor names to the same layout with their vesc ids
fn convertLayout(allocator: std.mem.Allocator, config: MonitoringConfig) ![][]u8 {
    const id_layout: [][]u8 = try allocator.alloc([]u8, config.layout.len);
    for (config.layout, id_layout) |str_row, *id_row| {
        id_row.* = try allocator.alloc(u8, str_row.len);

        for (str_row, id_row.*) |name, *id| {
            var found_name = false;
            for (config.vesc_configs) |vesc_conf| {
                if (std.mem.eql(u8, vesc_conf.name, name)) {
                    id.* = vesc_conf.can_id;
                    found_name = true;
                    break;
                }
            }
            if (!found_name) {
                panic("No Motor Named \"{s}\", double check the layout", .{name});
            }
        }
    }
    return id_layout;
}
