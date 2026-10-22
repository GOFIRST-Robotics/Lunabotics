const std = @import("std");
const Io = std.Io;
const io: Io = std.Options.debug_io;

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
var server: ?*fg.foxglove_websocket_server = undefined;
var channel_map: std.StringHashMap(fg_channel) = undefined;
var channel_mutex: std.Io.Mutex = .init;
var allocator: std.mem.Allocator = undefined;
var start_time: Io.Timestamp = undefined;

pub fn init(alloc: std.mem.Allocator) void {
    allocator = alloc;
    channel_map = .init(allocator);
    start_time = Io.Clock.now(.real, io);
    startServer();
}

pub fn deinit() void {
    channel_map.deinit();
    stopServer();
}

fn startServer() void {
    const options = fg.foxglove_server_options{
        .host = fgStr(host),
        .port = port,
    };
    const err = fg.foxglove_server_start(&options, &server);
    logFgError(err, "[Foxglove] Unable to start foxglove websocket server!");
}

fn stopServer() void {
    const err = fg.foxglove_server_stop(server);
    logFgError(err, "[Foxglove] Unable to stop foxglove websocket server!");
}

pub fn logMessage(topic: []const u8, msg: []const u8, level: LogLevel) void {
    const entry = try getOrPutChannelLocked(topic) catch return;
    if (!entry.found_existing) {
        entry.value_ptr.* = createLogChannel(topic);
    }
    const message = fg.foxglove_log{
        .level = @intFromEnum(level),
        .message = fgStr(msg),
        .timestamp = &timestamp(),
    };
    const err = fg.foxglove_channel_log_log(entry.value_ptr.*, &message, null, 0);
    logFgError(err, "[Foxglove] Unable to log log(message) data!");
}

pub fn logData(topic: []const u8, data: anytype) void {
    if (@typeInfo(@TypeOf(data)) != .@"struct") {
        @compileError("[Foxglove] Can only log struct types! Logging incorrect type to {}");
    }
    const entry = getOrPutChannelLocked(topic) catch return;
    if (!entry.found_existing) {
        entry.value_ptr.* = createRawChannel(topic);
    }
    const json = std.json.Stringify.valueAlloc(allocator, data, .{}) catch {
        logger.warn("[Foxglove] Unable to stringify data to JSON!", .{});
        return;
    };
    defer allocator.free(json);
    const err = fg.foxglove_channel_log(entry.value_ptr.*, json.ptr, json.len, null, 0);
    logFgError(err, "[Foxglove] Unable to log raw data!");
}

pub fn logPose(topic: []const u8, pose: fg.foxglove_pose) void {
    const entry = try getOrPutChannelLocked(topic) catch return;
    if (!entry.found_existing) {
        entry.value_ptr.* = createPoseInFrameChannel(topic);
    }
    const message = fg.foxglove_pose_in_frame{
        .frame_id = fgStr("map"),
        .pose = &pose,
        .timestamp = &timestamp(),
    };
    const err = fg.foxglove_channel_log_pose_in_frame(entry.value_ptr.*, &message, null, 0);
    logFgError(err, "[Foxglove] Unable to log pose in frame data!");
}

fn createLogChannel(topic: []const u8) fg_channel {
    var channel: fg_channel = undefined;
    const err = fg.foxglove_channel_create_log(fgStr(topic), null, &channel);
    logFgError(err, "[Foxglove] Unable to create log channel!");
    return channel;
}

fn createRawChannel(topic: []const u8) fg_channel {
    var channel: fg_channel = undefined;
    const err = fg.foxglove_raw_channel_create(fgStr(topic), fgStr("json"), null, null, null, &channel);
    logFgError(err, "[Foxglove] Unable to create raw channel!");
    return channel;
}

fn createPoseInFrameChannel(topic: []const u8) fg_channel {
    var channel: fg_channel = undefined;
    const err = fg.foxglove_channel_create_pose_in_frame(fgStr(topic), null, &channel);
    logFgError(err, "[Foxglove] Unable to create pose in frame channel!");
    return channel;
}

fn getOrPutChannelLocked(topic: []const u8) !std.StringHashMap(fg_channel).GetOrPutResult {
    try channel_mutex.lock(io);
    defer channel_mutex.unlock(io);
    const entry = channel_map.getOrPut(topic) catch |err| {
        logger.warn("[Foxglove] Unable to put value into channel hashmap for topic {s}!", .{topic});
        return err;
    };
    return entry;
}

fn fgStr(str: []const u8) fg.foxglove_string {
    return fg.foxglove_string{ .data = str.ptr, .len = str.len };
}

fn timestamp() fg.foxglove_timestamp {
    const current_time = Io.Timestamp.fromNanoseconds(Io.Clock.now(.real, io).nanoseconds - start_time.nanoseconds);
    const seconds: u32 = @intCast(current_time.toSeconds());
    const nanoseconds: u32 = @intCast(current_time.subDuration(Io.Duration.fromSeconds(seconds)).toNanoseconds());
    return .{ .sec = seconds, .nsec = nanoseconds };
}

fn logFgError(err: fg.foxglove_error, comptime str: []const u8) void {
    if (err != fg.FOXGLOVE_ERROR_OK) {
        logger.err(str ++ " Error {}.", .{err});
    }
}
