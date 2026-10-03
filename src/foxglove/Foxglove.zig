const std = @import("std");
const Io = std.Io;

const fg = @import("foxglove-sdk");

const Foxglove = @This();


const host = "0.0.0.0";
const port = 8765;

pub fn startServer() void {
    const options = fg.foxglove_server_options{
        .host = fg_str(host),
        .port = 8765,
    };
    var server: ?*fg.foxglove_websocket_server = undefined;
    _ = fg.foxglove_server_start(&options, &server);
}

// pub fn logMessage(topic: []const u8, msg: []const u8) void {}

pub fn testFG(init: std.process.Init) !void {
    std.debug.print("Hello", .{});
    const topic = fg_str("/Test");
    // const encoding = fg_str("json");
    // const schema: ?*fg.foxglove_schema = null;
    const context: ?*fg.foxglove_context = null;
    // const metadata: ?*fg.foxglove_channel_metadata = null;
    var channel: ?*const fg.foxglove_channel = undefined;
    _ = fg.foxglove_channel_create_log(topic, context, &channel);
    var buf: [20]u8 = undefined;
    var i: u8 = 0;
    while (true) {
        std.debug.print("{}\n", .{i});

        // Convert the integer to a string slice
        const num_str = try std.fmt.bufPrint(&buf, "{d}", .{i});
        // Log Data
        const message = fg.foxglove_log{
            .level = fg.FOXGLOVE_LOG_LEVEL_INFO,
            .message = fg_str(num_str),
        };
        _ = fg.foxglove_channel_log_log(channel, &message, null, 0);
        i += 1;
        if (i == 127) {
            i = 0;
        }
        try init.io.sleep(Io.Duration.fromSeconds(1), .real);
    }
}

fn fg_str(str: []const u8) fg.foxglove_string {
    return fg.foxglove_string{ .data = str.ptr, .len = str.len };
}
