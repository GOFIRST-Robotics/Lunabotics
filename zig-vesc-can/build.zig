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
}
