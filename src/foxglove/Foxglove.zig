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

fn fg_str(str: []const u8) fg.foxglove_string {
    return fg.foxglove_string{ .data = str.ptr, .len = str.len };
}
