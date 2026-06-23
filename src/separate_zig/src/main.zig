const std = @import("std");
const Io = std.Io;

const separate_zig = @import("separate_zig");
const interface = @import("interface");

pub fn main(init: std.process.Init) !void {
    _ = init;

    const node_handle = interface.init_ros2("zig_test_node");

    var sum: i32 = undefined;
    const success = interface.call_add_two_ints_service(node_handle, "zig_test/addition", 5, 7, &sum);
    if (success != 0) {
        std.debug.print("Failed to add\n", .{});
    }

    std.debug.print("5 + 7 = {d}\n", .{sum});
}
