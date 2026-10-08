const std = @import("std");
const Io = std.Io;

const fg = @import("foxglove-sdk");

pub const LogLevel = enum(u8) {
    UNKNOWN = fg.FOXGLOVE_LOG_LEVEL_UNKNOWN,
    DEBUG = fg.FOXGLOVE_LOG_LEVEL_DEBUG,
    INFO = fg.FOXGLOVE_LOG_LEVEL_INFO,
    WARNING = fg.FOXGLOVE_LOG_LEVEL_WARNING,
    ERROR = fg.FOXGLOVE_LOG_LEVEL_ERROR,
    FATAL = fg.FOXGLOVE_LOG_LEVEL_FATAL,
};
const fg_channel = ?*const fg.foxglove_channel;

const host = "0.0.0.0";
const port = 8765;
var channel_map: std.StringHashMap(fg_channel) = undefined;
var io_ref: Io = undefined;
var start_time: Io.Timestamp = undefined;

pub fn init(allocator: std.mem.Allocator, io: Io) void {
    channel_map = .init(allocator);
    io_ref = io;
    start_time = Io.Clock.now(.real, io_ref);
}

pub fn startServer() void {
    const options = fg.foxglove_server_options{
        .host = fg_str(host),
        .port = 8765,
    };
    var server: ?*fg.foxglove_websocket_server = undefined;
    _ = fg.foxglove_server_start(&options, &server);
}

pub fn logMessage(topic: []const u8, msg: []const u8, level: LogLevel) void {
    const entry = channel_map.getOrPut(topic) catch {
        std.debug.print("[Foxglove] Unable to put value into channel hashmap!", .{});
        return;
    };
    if (!entry.found_existing) {
        entry.value_ptr.* = createLogChannel(topic);
    }
    const message = fg.foxglove_log{
        .level = @intFromEnum(level),
        .message = fg_str(msg),
        .timestamp = &timestamp(),
    };
    _ = fg.foxglove_channel_log_log(entry.value_ptr.*, &message, null, 0);
}

pub fn logPose(topic: []const u8, x: f64, y: f64, z: f64) void {
    const entry = channel_map.getOrPut(topic) catch {
        std.debug.print("[Foxglove] Unable to put value into channel hashmap!", .{});
        return;
    };
    if (!entry.found_existing) {
        entry.value_ptr.* = createPoseChannel(topic);
    }
    const message = fg.foxglove_pose{
        .position = &.{ .x = x, .y = y, .z = z },
        .orientation = &.{ .x = 0, .y = 0, .z = 0, .w = 0 },
    };
    _ = fg.foxglove_channel_log_pose(entry.value_ptr.*, &message, null, 0);
}

fn createLogChannel(topic: []const u8) fg_channel {
    var channel: fg_channel = undefined;
    _ = fg.foxglove_channel_create_log(fg_str(topic), null, &channel);
    return channel;
}

fn createPoseChannel(topic: []const u8) fg_channel {
    var channel: fg_channel = undefined;
    _ = fg.foxglove_channel_create_pose(fg_str(topic), null, &channel);
    return channel;
}

fn fg_str(str: []const u8) fg.foxglove_string {
    return fg.foxglove_string{ .data = str.ptr, .len = str.len };
}

fn timestamp() fg.foxglove_timestamp {
    const time = Io.Timestamp.fromNanoseconds(Io.Clock.now(.real, io_ref).nanoseconds - start_time.nanoseconds);
    const seconds: u32 = @intCast(time.toSeconds());
    return .{ .sec = seconds };
}

// pub fn testFG(init: std.process.Init) !void {
//     std.debug.print("Hello", .{});
//     const topic = fg_str("/Test");
//     // const encoding = fg_str("json");
//     // const schema: ?*fg.foxglove_schema = null;
//     const context: ?*fg.foxglove_context = null;
//     // const metadata: ?*fg.foxglove_channel_metadata = null;
//     var channel: fg_channel = undefined;
//     _ = fg.foxglove_channel_create_log(topic, context, &channel);
//     var buf: [20]u8 = undefined;
//     var i: u8 = 0;
//     while (true) {
//         std.debug.print("{}\n", .{i});

//         // Convert the integer to a string slice
//         const num_str = try std.fmt.bufPrint(&buf, "{d}", .{i});
//         // Log Data
//         const message = fg.foxglove_log{
//             .level = fg.FOXGLOVE_LOG_LEVEL_INFO,
//             .message = fg_str(num_str),
//         };
//         _ = fg.foxglove_channel_log_log(channel, &message, null, 0);
//         i += 1;
//         if (i == 127) {
//             i = 0;
//         }
//         try init.io.sleep(Io.Duration.fromSeconds(1), .real);
//     }
// }
