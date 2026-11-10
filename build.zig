const std = @import("std");
const Translator = @import("translate_c").Translator;

pub fn build(b: *std.Build) void {
    const translate_c = b.dependency("translate_c", .{});
    const foxglove_header_path: std.Build.LazyPath = .{ .cwd_relative = "/usr/local/include/foxglove/foxglove-c.h" };

    // Force Formatting
    const format_requirement = b.addFmt(.{
        .check = true,
        // check everything
        .paths = &.{.{ .cwd_relative = "." }},
    });

    b.getInstallStep().dependOn(&format_requirement.step);

    // -----------------------------------------------
    // Local Build for Debuging Purposes
    // -----------------------------------------------
    const local_target = b.standardTargetOptions(.{
        .default_target = .{ .os_tag = .linux, .abi = .gnu },
    });
    const debug_optimizations: std.builtin.OptimizeMode = .Debug;

    const local_zig_vesc_can_dep = b.dependency(
        "zig_vesc_can",
        .{
            .target = local_target,
            .optimize = debug_optimizations,
        },
    );

    const local_zig_vesc_can_mod = local_zig_vesc_can_dep.module("zig-vesc-can");

    const local_config_mod = b.createModule(.{
        .optimize = debug_optimizations,
        .root_source_file = b.path("src/MFR/debug_config.zig"),
    });

    const local_foxglove_translator: Translator = .init(translate_c, .{
        .c_source_file = foxglove_header_path,
        .target = local_target,
        .optimize = debug_optimizations,
        .link_system_libs = &.{
            .{ .name = "foxglove" },
        },
    });

    const debug_MFR = b.addModule(
        "debug_MFR",
        .{
            .root_source_file = b.path("src/root.zig"),
            .target = local_target,
            .imports = &.{
                .{ .name = "config", .module = local_config_mod },
                .{ .name = "zig-vesc-can", .module = local_zig_vesc_can_mod },
                .{ .name = "foxglove-sdk", .module = local_foxglove_translator.mod },
            },
        },
    );
    const foxglove_x86_path: std.Build.LazyPath = .{ .cwd_relative = "/usr/local/lib/foxglove/x86_64/" };
    debug_MFR.addLibraryPath(foxglove_x86_path);
    debug_MFR.addRPath(foxglove_x86_path);

    const local_build = b.addExecutable(.{
        .name = "MFR_local",
        .root_module = b.createModule(.{
            .root_source_file = b.path("src/MFR/main.zig"),
            .target = local_target,
            .optimize = debug_optimizations,
            .imports = &.{
                .{ .name = "MFR", .module = debug_MFR },
            },
            .single_threaded = false,
        }),
        // overide becuase zig's system does not support bleading edge gcc
        .use_lld = true,
        .use_llvm = true,
    });

    const local_install = b.addInstallArtifact(local_build, .{});
    const local_build_step = b.step("local", "Just Compile Local Build");
    local_build_step.dependOn(&local_install.step);
    local_install.step.dependOn(&format_requirement.step);
    b.getInstallStep().dependOn(local_build_step);

    // -----------------------------------------------
    // Jetson Build
    // -----------------------------------------------
    const jetson_target = b.resolveTargetQuery(.{
        .cpu_arch = .aarch64,
        .os_tag = .linux,
        .abi = .gnu,
    });
    const jetson_optimization: std.builtin.OptimizeMode = .Debug;

    const jetson_zig_vesc_can_dep = b.dependency(
        "zig_vesc_can",
        .{
            .target = jetson_target,
            .optimize = jetson_optimization,
        },
    );

    const jetson_zig_vesc_can_mod = jetson_zig_vesc_can_dep.module("zig-vesc-can");

    const jetson_config_mod = b.createModule(.{
        .optimize = jetson_optimization,
        .root_source_file = b.path("src/MFR/jetson_config.zig"),
    });

    const jetson_foxglove_translator: Translator = .init(translate_c, .{
        .c_source_file = foxglove_header_path,
        .target = jetson_target,
        .optimize = jetson_optimization,
        .link_system_libs = &.{
            .{ .name = "foxglove" },
        },
    });

    const jetson_MFR = b.addModule(
        "jetson_MFR",
        .{
            .root_source_file = b.path("src/root.zig"),
            .target = jetson_target,
            .imports = &.{
                .{ .name = "config", .module = jetson_config_mod },
                .{ .name = "zig-vesc-can", .module = jetson_zig_vesc_can_mod },
                .{ .name = "foxglove-sdk", .module = jetson_foxglove_translator.mod },
            },
        },
    );
    const foxglove_aarch64_path: std.Build.LazyPath = .{ .cwd_relative = "/usr/local/lib/foxglove/aarch64/" };
    jetson_MFR.addLibraryPath(foxglove_aarch64_path);
    jetson_MFR.addRPath(foxglove_aarch64_path);

    const jetson_build = b.addExecutable(.{
        .name = "MFR_jetson",
        .root_module = b.createModule(.{
            .root_source_file = b.path("src/MFR/main.zig"),
            .target = jetson_target,
            .optimize = jetson_optimization,
            .imports = &.{
                .{ .name = "MFR", .module = jetson_MFR },
            },
            .single_threaded = false,
        }),
        // overide becuase zig's system does not support bleading edge gcc
        .use_lld = true,
        .use_llvm = true,
    });

    const jetson_install = b.addInstallArtifact(jetson_build, .{});
    const jetson_build_step = b.step("jetson", "Just Compile Jetson Build");
    jetson_build_step.dependOn(&jetson_install.step);
    jetson_install.step.dependOn(&format_requirement.step);
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
    client_install.step.dependOn(&format_requirement.step);
    b.getInstallStep().dependOn(client_build_step);

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

    const test_step = b.step("test", "Run tests");
    test_step.dependOn(&run_mod_tests.step);
    test_step.dependOn(&run_exe_tests.step);
}
