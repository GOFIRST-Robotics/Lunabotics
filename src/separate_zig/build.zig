const std = @import("std");

pub fn build(b: *std.Build) void {
    const target = b.standardTargetOptions(.{});
    const optimize = b.standardOptimizeOption(.{});

    const translate_c = b.addTranslateC(.{
        .root_source_file = b.path("src/interface.h"),
        .target = target,
        .optimize = optimize,
    });

    const mod = b.addModule("separate_zig", .{
        .root_source_file = b.path("src/root.zig"),
        .target = target,
        .imports = &.{},
        // .link_libcpp = true,
        .link_libc = true,
    });

    mod.addObjectFile(b.path("build/libros_bridge.a"));

    mod.addLibraryPath(.{ .cwd_relative = "/opt/ros/humble/lib" });

    mod.addLibraryPath(.{ .cwd_relative = "/workspaces/isaac_ros-dev/install/rovr_interfaces/lib" });
    mod.linkSystemLibrary("rovr_interfaces__rosidl_typesupport_c", .{});
    mod.linkSystemLibrary("rovr_interfaces__rosidl_generator_c", .{});

    mod.addObjectFile(.{ .cwd_relative = "/usr/lib/x86_64-linux-gnu/libstdc++.so.6" });

    mod.linkSystemLibrary("rclcpp", .{});
    mod.linkSystemLibrary("rcl", .{});
    mod.linkSystemLibrary("rcutils", .{});
    mod.linkSystemLibrary("rosidl_typesupport_cpp", .{});
    mod.linkSystemLibrary("rovr_interfaces__rosidl_typesupport_c", .{});
    mod.linkSystemLibrary("rovr_interfaces__rosidl_generator_c", .{});
    mod.linkSystemLibrary("rovr_interfaces__rosidl_typesupport_cpp", .{});

    mod.linkSystemLibrary("gcc_s", .{});

    const exe = b.addExecutable(.{
        .name = "separate_zig",
        .root_module = b.createModule(.{
            .root_source_file = b.path("src/main.zig"),
            .target = target,
            .optimize = optimize,
            .imports = &.{
                .{ .name = "separate_zig", .module = mod },
                .{ .name = "interface", .module = translate_c.createModule() },
            },
        }),
    });

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
    });

    const run_mod_tests = b.addRunArtifact(mod_tests);

    const exe_tests = b.addTest(.{
        .root_module = exe.root_module,
    });

    const run_exe_tests = b.addRunArtifact(exe_tests);

    const test_step = b.step("test", "Run tests");
    test_step.dependOn(&run_mod_tests.step);
    test_step.dependOn(&run_exe_tests.step);
}
