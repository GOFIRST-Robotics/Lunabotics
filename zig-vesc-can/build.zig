const std = @import("std");

pub fn build(b: *std.Build) void {
    const target = b.standardTargetOptions(.{});
    const optimize = b.standardOptimizeOption(.{});

    const translate_c = b.addTranslateC(.{
        .root_source_file = b.path("src/can.h"),
        .target = target,
        .optimize = optimize,
    });
    const can_module = translate_c.createModule();

    const mod = b.addModule("zig-vesc-can", .{
        .root_source_file = b.path("src/root.zig"),
        .target = target,
        .imports = &.{.{
            .name = "can.h",
            .module = can_module,
        }},
    });

    // -----------------------------------------------
    // Tests For Custom Vesc CAN messages
    // -----------------------------------------------
    const vesc_comm_tests_run = vesc_comm_tests_run: {
        // !!! IMPORTANT !!!
        // This module must be used carefully as it's code is under the GPLv3
        // Any code that uses this module must also be under the GPLv3
        // Since the test file uses this module only it should need to be under the GPLv3
        const vesc_comm_can = b.addTranslateC(.{
            .root_source_file = b.path("src/vesc_comm_tests/comm_can.h"),
            .link_libc = true,
            .target = target,
            .optimize = optimize,
        });

        const vesc_comm_can_module = vesc_comm_can.createModule();

        const zig_vesc_can = b.addModule("zig_vesc_can_tests", .{
            .root_source_file = b.path("src/vesc_comm_tests/vesc_comm_tests.zig"),
            .imports = &.{
                .{ .name = "comm_can", .module = vesc_comm_can_module },
                .{ .name = "zig-vesc-can", .module = mod },
            },
            .optimize = optimize,
            .target = target,
            .link_libc = true,
        });

        zig_vesc_can.addCSourceFiles(.{
            .root = b.path("src/vesc_comm_tests/"),
            .files = &.{ "buffer.c", "comm_can.c" },
            .flags = &.{"--std=c99"},
            .language = .c,
        });

        const zig_vesc_can_test = b.addTest(.{
            .root_module = zig_vesc_can,
            .use_lld = true,
            .use_llvm = true,
        });

        const run_tests = b.addRunArtifact(zig_vesc_can_test);

        break :vesc_comm_tests_run run_tests;
    };

    const exe = b.addExecutable(.{
        .name = "zig_vesc_can",
        .root_module = b.createModule(.{
            .root_source_file = b.path("src/main.zig"),
            .target = target,
            .optimize = optimize,
            .imports = &.{
                .{ .name = "zig-vesc-can", .module = mod },

                .{
                    .name = "can.h",
                    .module = can_module,
                },
            },
            .link_libc = true,
        }),
    });

    exe.use_lld = true;
    exe.use_llvm = true;

    b.installArtifact(exe);

    const run_step = b.step("run", "Run the app");

    const run_cmd = b.addRunArtifact(exe);
    run_step.dependOn(&run_cmd.step);

    run_cmd.step.dependOn(b.getInstallStep());

    if (b.args) |args| {
        run_cmd.addArgs(args);
    }

    const mod_tests = b.addTest(.{
        .root_module = mod,
        .use_llvm = true,
        .use_lld = true,
    });

    mod_tests.root_module.addImport("can", translate_c.createModule());

    const run_mod_tests = b.addRunArtifact(mod_tests);

    const exe_tests = b.addTest(.{
        .root_module = exe.root_module,
        .use_lld = true,
        .use_llvm = true,
    });

    exe_tests.root_module.addImport("can", translate_c.createModule());

    const run_exe_tests = b.addRunArtifact(exe_tests);

    const test_step = b.step("test", "Run tests");
    test_step.dependOn(&run_mod_tests.step);
    test_step.dependOn(&run_exe_tests.step);
    test_step.dependOn(&vesc_comm_tests_run.step);
}
