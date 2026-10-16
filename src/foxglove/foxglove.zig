const std = @import("std");
const Io = std.Io;

const fg = @import("foxglove-sdk");

const logger = std.log.scoped(.foxglove);

pub const LogLevel = enum(u8) {
    UNKNOWN = fg.FOXGLOVE_LOG_LEVEL_UNKNOWN,
    DEBUG = fg.FOXGLOVE_LOG_LEVEL_DEBUG,
    INFO = fg.FOXGLOVE_LOG_LEVEL_INFO,
    WARNING = fg.FOXGLOVE_LOG_LEVEL_WARNING,
    ERROR = fg.FOXGLOVE_LOG_LEVEL_ERROR,
    FATAL = fg.FOXGLOVE_LOG_LEVEL_FATAL,
};
const fg_channel: type = ?*const fg.foxglove_channel;

const host = "0.0.0.0";
const port = 8765;
var channel_map: std.StringHashMap(fg_channel) = undefined;
var json_string_builder: std.Io.Writer.Allocating = undefined;
var io_ref: Io = undefined;
var start_time: Io.Timestamp = undefined;

pub fn init(allocator: std.mem.Allocator, io: Io) void {
    channel_map = .init(allocator);
    json_string_builder = .init(allocator);
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
        logger.warn("[Foxglove] Unable to put value into channel hashmap!", .{});
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

pub fn logData(topic: []const u8, data: anytype) void {
    if (@typeInfo(@TypeOf(data)) != .@"struct") {
        @compileError("[Foxglove] Can only log struct types! Logging incorrect type to {}");
    }
    const entry = channel_map.getOrPut(topic) catch {
        logger.warn("[Foxglove] Unable to put value into channel hashmap!", .{});
        return;
    };
    if (!entry.found_existing) {
        entry.value_ptr.* = createRawChannel(topic);
    }
    std.json.Stringify.value(data, .{}, &json_string_builder.writer) catch {
        logger.warn("[Foxglove] Unable to stringify data to JSON!", .{});
        return;
    };
    const json = json_string_builder.written();
    json_string_builder.clearRetainingCapacity();
    _ = fg.foxglove_channel_log(entry.value_ptr.*, json.ptr, json.len, null, 0);
}

pub fn logPose(topic: []const u8, pose: fg.foxglove_pose) void {
    const entry = channel_map.getOrPut(topic) catch {
        logger.warn("[Foxglove] Unable to put value into channel hashmap!", .{});
        return;
    };
    if (!entry.found_existing) {
        entry.value_ptr.* = createPoseInFrameChannel(topic);
    }
    const message = fg.foxglove_pose_in_frame{
        .frame_id = fg_str("map"),
        .pose = &pose,
        .timestamp = &timestamp(),
    };
    _ = fg.foxglove_channel_log_pose_in_frame(entry.value_ptr.*, &message, null, 0);
}

fn createLogChannel(topic: []const u8) fg_channel {
    var channel: fg_channel = undefined;
    _ = fg.foxglove_channel_create_log(fg_str(topic), null, &channel);
    return channel;
}

fn createRawChannel(topic: []const u8) fg_channel {
    var channel: fg_channel = undefined;
    _ = fg.foxglove_raw_channel_create(fg_str(topic), fg_str("json"), null, null, null, &channel);
    return channel;
}

fn createPoseInFrameChannel(topic: []const u8) fg_channel {
    var channel: fg_channel = undefined;
    _ = fg.foxglove_channel_create_pose_in_frame(fg_str(topic), null, &channel);
    return channel;
}

fn fg_str(str: []const u8) fg.foxglove_string {
    return fg.foxglove_string{ .data = str.ptr, .len = str.len };
}

fn timestamp() fg.foxglove_timestamp {
    const current_time = Io.Timestamp.fromNanoseconds(Io.Clock.now(.real, io_ref).nanoseconds - start_time.nanoseconds);
    const seconds: u32 = @intCast(current_time.toSeconds());
    const nanoseconds: u32 = @intCast(current_time.subDuration(Io.Duration.fromSeconds(seconds)).toNanoseconds());
    return .{ .sec = seconds, .nsec = nanoseconds };
}
