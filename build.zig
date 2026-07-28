const std = @import("std");

pub fn build(b: *std.Build) void {

    // -----------------------------------------------
    // Local Build for Debuging Purposes
    // -----------------------------------------------
    const local_target = b.standardTargetOptions(.{ .default_target = .{ .os_tag = .linux } });

    const debug_optimizations: std.builtin.OptimizeMode = .Debug;

    const jetson_target = b.resolveTargetQuery(.{ .cpu_arch = .aarch64, .os_tag = .linux, .abi = .gnu });

    const local_config_mod = b.createModule(.{
        .optimize = debug_optimizations,
        .root_source_file = b.path("src/MFR/debug_config.zig"),
    });

    const local_can_c = b.addTranslateC(.{
        .root_source_file = b.path("src/can_bus/socket_can.h"),
        .target = local_target,
        .optimize = debug_optimizations,
    });

    const debug_MFR = b.addModule(
        "MFR",
        .{
            .root_source_file = b.path("src/root.zig"),
            .target = local_target,
            .imports = &.{
                .{ .name = "config", .module = local_config_mod },
                .{ .name = "socket_can", .module = local_can_c.createModule() },
                // .{ .name = "config", .module = local_config_mod },
            },
        },
    );

    const local_build = b.addExecutable(.{
        .name = "MFR_local",
        .root_module = b.createModule(.{
            .root_source_file = b.path("src/MFR/main.zig"),
            .target = local_target,
            .optimize = debug_optimizations,
            .imports = &.{
                .{ .name = "MFR", .module = debug_MFR },
            },
            .single_threaded = false, // TODO: fully understand this
        }),
        // overide becuase zig's system does not support bleading edge gcc
        .use_lld = true,
        .use_llvm = true,
    });

    const local_install = b.addInstallArtifact(local_build, .{});
    const local_build_step = b.step("local", "Just Compile Local Build");
    local_build_step.dependOn(&local_install.step);
    b.getInstallStep().dependOn(local_build_step);

    // -----------------------------------------------
    // Jetson Build
    // -----------------------------------------------

    const jetson_optimization: std.builtin.OptimizeMode = .Debug;

    const jetson_config_mod = b.createModule(.{
        .optimize = jetson_optimization,
        .root_source_file = b.path("src/MFR/jetson_config.zig"),
    });

    const jetson_can_c = b.addTranslateC(.{
        .root_source_file = b.path("src/can_bus/socket_can.h"),
        .target = jetson_target,
        .optimize = jetson_optimization,
    });

    const jetson_MFR = b.addModule(
        "MFR",
        .{
            .root_source_file = b.path("src/root.zig"),
            .target = jetson_target,
            .imports = &.{
                .{ .name = "config", .module = jetson_config_mod },
                .{ .name = "socket_can", .module = jetson_can_c.createModule() },
            },
        },
    );

    const jetson_build = b.addExecutable(.{
        .name = "MFR_jetson",
        .root_module = b.createModule(.{
            .root_source_file = b.path("src/MFR/main.zig"),
            .target = jetson_target,
            .optimize = jetson_optimization,
            .imports = &.{
                .{ .name = "MFR", .module = jetson_MFR },
            },
            .single_threaded = false, // TODO: fully understand this
        }),
        // overide becuase zig's system does not support bleading edge gcc
        .use_lld = true,
        .use_llvm = true,
    });

    const jetson_install = b.addInstallArtifact(jetson_build, .{});
    const jetson_build_step = b.step("jetson", "Just Compile Jetson Build");
    jetson_build_step.dependOn(&jetson_install.step);
    b.getInstallStep().dependOn(jetson_build_step);

    // -----------------------------------------------
    // Build for Control Station Client
    // -----------------------------------------------

    const control_satation_optimization: std.builtin.OptimizeMode = .Debug;

    const input_headers_c = b.addTranslateC(.{
        .root_source_file = b.path("src/controller_com/input_headers.h"),
        .target = local_target,
        .optimize = control_satation_optimization,
    });

    const client_build = b.addExecutable(.{
        .name = "client",
        .root_module = b.createModule(.{
            .root_source_file = b.path("src/controller_com/Client.zig"),
            .target = local_target,
            .optimize = control_satation_optimization,
            .imports = &.{
                .{ .name = "input_headers", .module = input_headers_c.createModule() },
            },
            .link_libc = true,
        }),
        // overide becuase zig's system does not support bleading edge gcc
        .use_lld = true,
        .use_llvm = true,
    });

    const client_install = b.addInstallArtifact(client_build, .{});
    const client_build_step = b.step("client", "Just Compile Control Station Client");
    client_build_step.dependOn(&client_install.step);
    b.getInstallStep().dependOn(client_build_step);

    // -----------------------------------------------
    // CAN Monitoring Build for both Jetson and Local Build
    // -----------------------------------------------

    const can_monitor_optimization: std.builtin.OptimizeMode = .Debug;

    const local_monitor_build = b.addExecutable(.{
        .name = "local_can_monitor",
        .root_module = b.createModule(
            .{
                .root_source_file = b.path("src/can_bus/monitor.zig"),
                .target = local_target,
                .optimize = can_monitor_optimization,
                .imports = &.{
                    .{ .name = "MFR", .module = debug_MFR },
                },
            },
        ),
    });

    const local_monitor_install = b.addInstallArtifact(local_monitor_build, .{});
    local_build_step.dependOn(&local_monitor_install.step);

    const jetson_monitor_build = b.addExecutable(.{
        .name = "jetson_can_monitor",
        .root_module = b.createModule(
            .{
                .root_source_file = b.path("src/can_bus/monitor.zig"),
                .target = local_target,
                .optimize = can_monitor_optimization,
                .imports = &.{
                    .{ .name = "MFR", .module = jetson_MFR },
                },
            },
        ),
    });

    const jetson_monitor_install = b.addInstallArtifact(jetson_monitor_build, .{});
    jetson_build_step.dependOn(&jetson_monitor_install.step);

    // -----------------------------------------------
    // Tests For Custom Vesc CAN messages
    // -----------------------------------------------
    const zig_vesc_can_tests = create_test_step: {
        // !!! IMPORTANT !!!
        // This module must be used carefully as its code is under the GPLv3
        // Any code that uses this module must also be under the GPLv3
        // Since the test file uses this module only it should need to be under the GPLv3
        const vesc_comm_can = b.addTranslateC(.{
            .target = local_target,
            .root_source_file = b.path("src/can_bus/vesc_comm_tests/comm_can.h"),
            .link_libc = true,
            .optimize = .Debug,
        });
        const vesc_comm_can_module = vesc_comm_can.createModule();
        vesc_comm_can_module.addCSourceFiles(.{
            .root = b.path("src/can_bus/vesc_comm_tests"),
            .files = &.{
                "comm_can.c",
                "buffer.c",
            },
            .flags = &.{"-std=c99"},
            .language = .c,
        });

        const zig_vesc_can = b.addModule("zig_vesc_can_tests", .{
            .root_source_file = b.path("src/can_bus/vesc_comm_tests/vesc_comm_tests.zig"),
            .imports = &.{
                .{ .name = "comm_can", .module = vesc_comm_can_module },
                .{ .name = "MFR", .module = debug_MFR },
            },
            .optimize = .Debug,
            .target = local_target,
            .link_libc = true,
        });
        break :create_test_step b.addTest(.{
            .root_module = zig_vesc_can,
            .use_lld = true,
            .use_llvm = true,
        });
    };

    const run_step = b.step("run", "Run the app");

    const run_cmd = b.addRunArtifact(local_build);
    run_step.dependOn(&run_cmd.step);

    run_cmd.step.dependOn(b.getInstallStep());

    if (b.args) |args| {
        run_cmd.addArgs(args);
    }

    const mod_tests = b.addTest(.{
        .root_module = debug_MFR,
        .use_lld = true,
        .use_llvm = true,
    });

    const run_mod_tests = b.addRunArtifact(mod_tests);

    const exe_tests = b.addTest(.{
        .root_module = local_build.root_module,
    });

    const run_exe_tests = b.addRunArtifact(exe_tests);

    const zig_vesc_can_test_runs = b.addRunArtifact(zig_vesc_can_tests);

    const test_step = b.step("test", "Run tests");
    test_step.dependOn(&run_mod_tests.step);
    test_step.dependOn(&run_exe_tests.step);
    test_step.dependOn(&zig_vesc_can_test_runs.step);
}
