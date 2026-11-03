const std = @import("std");
const protocol = @import("protocol.zig");
const Io = std.Io;
const net = Io.net;
const MechanismNode = @import("../robot_logic/MechanismNode.zig");

const ServerNode = @This();

const logger = std.log.scoped(.server);

const server_address: net.IpAddress = .{ .ip4 = .{ .port = protocol.port, .bytes = .{ 0, 0, 0, 0 } } };

const connection_timeout = Io.Duration.fromMilliseconds(500); // How long to wait before timeout

pub const ConnectionState = enum(u8) { lost, connected };

io: Io,
socket: net.Socket,
loops: u64,

connection_state: std.atomic.Value(ConnectionState),
link: Link,

previous_controller_data: protocol.ControllerData,
last_controller_time: Io.Timestamp,
controller_up: bool,

previous_stream_deck_data: protocol.StreamDeckData,
last_stream_deck_time: Io.Timestamp,
stream_deck_up: bool,

pub const inputType = struct {};

pub const outputType = struct {
    mechanism_instructions: *MechanismNode.Instructions,
    pub fn linkBuffer(self: *@This(), mech: *MechanismNode.inputType) void {
        self.mechanism_instructions = &mech.instructions;
    }
};

/// Connection-timeout bookkeeping, separated from the socket so it can be driven with fake time
pub const Link = struct {
    state: ConnectionState = .lost,
    last_valid_packet_time: Io.Timestamp = .zero,
    lost_since: ?Io.Timestamp = null,

    /// became_connected carries the downtime (null on the first connection)
    pub const Event = union(enum) { became_connected: ?Io.Duration, became_lost: Io.Duration };

    pub fn update(self: *Link, current_time: Io.Timestamp, got_valid_packet: bool) ?Event {
        const time_since_valid_packet = self.last_valid_packet_time.durationTo(current_time);

        if (got_valid_packet) {
            self.last_valid_packet_time = current_time;
            if (self.state == .connected) return null;
            self.state = .connected;
            return .{ .became_connected = if (self.lost_since) |lost_since| lost_since.durationTo(current_time) else null };
        } else if (self.state == .connected and time_since_valid_packet.nanoseconds > connection_timeout.nanoseconds) {
            self.lost_since = self.last_valid_packet_time;
            self.state = .lost;
            return .{ .became_lost = time_since_valid_packet };
        }
        return null;
    }
};

pub fn init(init_only_io: Io) @This() {
    return initAt(init_only_io, server_address);
}

fn initAt(init_only_io: Io, address: net.IpAddress) @This() {
    var ret: @This() = undefined;
    ret.io = undefined;
    ret.loops = 0;

    ret.connection_state = .init(.lost);
    ret.link = .{};

    ret.previous_controller_data = .{};
    ret.previous_stream_deck_data = .{};
    ret.controller_up = false;
    ret.stream_deck_up = false;

    // There are no obvious defaults for these
    // but logic wise it makes sense to just assume the last data was too long ago
    ret.last_controller_time = .zero;
    ret.last_stream_deck_time = .zero;

    ret.socket = address.bind(init_only_io, .{ .protocol = .udp, .mode = .dgram }) catch @panic("Failed to open UDP controller connection");
    logger.info("Successfully bound server to port {d}", .{protocol.port});

    return ret;
}

pub fn deinit(self: *@This()) void {
    _ = self;
}

pub fn connectionState(self: *const @This()) ConnectionState {
    return self.connection_state.load(.seq_cst);
}

fn updateConnectionState(self: *@This(), current_time: Io.Timestamp, got_valid_packet: bool) void {
    switch (self.link.update(current_time, got_valid_packet) orelse return) {
        .became_connected => |downtime| {
            if (downtime) |down| {
                logger.info("Control station connection restored after {f} down", .{down});
            } else {
                logger.info("Control station connected", .{});
            }
            self.connection_state.store(.connected, .seq_cst);
        },
        .became_lost => |time_since_valid_packet| {
            logger.warn("Lost connection to control station; no valid packet for {f}", .{time_since_valid_packet});
            self.connection_state.store(.lost, .seq_cst);
        },
    }
}

pub fn update(self: *@This(), input: *inputType, output: *outputType) void {
    _ = input;
    // logger.info("{any}", .{&self.io});

    var dgram_buf: [1024]u8 = undefined;

    const receive_timeout: Io.Timeout = switch (self.connection_state.load(.seq_cst)) {
        .connected => .{ .deadline = self.link.last_valid_packet_time.addDuration(connection_timeout).withClock(.awake) },
        .lost => .{ .duration = .{ .raw = connection_timeout, .clock = .awake } },
    };
    const receive_result = self.socket.receiveTimeout(self.io, &dgram_buf, receive_timeout);

    const current_time = Io.Timestamp.now(self.io, .awake);
    // Io.Timestamp.now gives back zero when getting the time fails
    if (current_time.nanoseconds == 0) {
        logger.warn("Failed to get current time from the awake clock", .{});
        return;
    }

    const maybe_message: ?protocol.Message = parse: {
        const dgram_info = receive_result catch |err| switch (err) {
            error.Timeout => break :parse null,
            else => {
                logger.warn("Failed to read socket, {s}", .{@errorName(err)});
                break :parse null;
            },
        };
        const dgram_data = dgram_info.data;

        break :parse protocol.parseDgram(dgram_data) catch |err| {
            logger.warn("Dropped dgram of {d} bytes; {s}", .{ dgram_data.len, @errorName(err) });
            break :parse null;
        };
    };

    self.updateConnectionState(current_time, maybe_message != null);

    const message = maybe_message orelse return;
    const message_time = message.timestamp;

    const message_age = message_time.durationTo(current_time);
    if (message_age.nanoseconds > Io.Duration.fromSeconds(1).nanoseconds) {
        logger.info("Got stale data, {f} old\ncurrent time: {d}ns\nmessage time: {d}ns", .{
            message_age,
            current_time,
            message_time,
        });
        return;
    }

    switch (message.payload) {
        .controller => |controller_data| {
            // if this is true then this dgram is either old data or a duplicate
            if (message_time.nanoseconds < self.last_controller_time.nanoseconds) {
                return;
            }

            if (controller_data.d_pad.up != self.previous_controller_data.d_pad.up) {
                if (controller_data.d_pad.up) {
                    logger.info("D pad up pressed", .{});
                } else {
                    logger.info("D pad up unpressed", .{});
                }
            }

            self.previous_controller_data = controller_data;
            self.last_controller_time = message_time;
            self.controller_up = true;
        },
        .controller_down => {
            self.previous_controller_data = .{};
            self.last_controller_time = message_time;
            self.controller_up = false;
        },
        .stream_deck => |stream_deck_data| {
            // if this is true then this dgram is either old data or a duplicate
            if (message_time.nanoseconds < self.last_stream_deck_time.nanoseconds) {
                return;
            }

            if (stream_deck_data.btn_0 != self.previous_stream_deck_data.btn_0) {
                if (stream_deck_data.btn_0) {
                    logger.info("btn 0 pressed", .{});
                } else {
                    logger.info("btn 0 unpressed", .{});
                }
            }

            self.previous_stream_deck_data = stream_deck_data;
            self.last_stream_deck_time = message_time;
            self.stream_deck_up = true;
        },
        .stream_deck_down => {
            self.previous_stream_deck_data = .{};
            self.last_controller_time = message_time;
            self.stream_deck_up = false;
        },
        .ending_connection => {
            return;
        },
    }

    if (self.controller_up) {
        if (self.previous_controller_data.r3) {
            output.mechanism_instructions.extend = true;
        } else if (self.previous_controller_data.l3) {
            output.mechanism_instructions.extend = false;
        } else {
            output.mechanism_instructions.extend = null;
        }

        if (self.previous_controller_data.buttons.east) {
            output.mechanism_instructions.plundge_rpm = 10;
        } else if (self.previous_controller_data.buttons.north) {
            output.mechanism_instructions.plundge_rpm = -10;
        } else {
            output.mechanism_instructions.plundge_rpm = 0;
        }
    } else {
        // turn everything off
        output.mechanism_instructions.* = .{};
    }
}

test "link transitions: connect, timeout boundary, lost once, reconnect with downtime" {
    const t0: Io.Timestamp = .fromNanoseconds(1_000_000_000);
    var link: Link = .{};
    try std.testing.expectEqual(null, link.update(t0, false));
    try std.testing.expectEqual(Link.Event{ .became_connected = null }, link.update(t0, true));
    try std.testing.expectEqual(null, link.update(t0.addDuration(connection_timeout), false));
    const t_lost = t0.addDuration(connection_timeout).addDuration(.fromNanoseconds(1));
    try std.testing.expectEqual(Link.Event{ .became_lost = t0.durationTo(t_lost) }, link.update(t_lost, false));
    try std.testing.expectEqual(null, link.update(t_lost.addDuration(.fromSeconds(1)), false));
    const t1 = t0.addDuration(.fromSeconds(5));
    try std.testing.expectEqual(Link.Event{ .became_connected = t0.durationTo(t1) }, link.update(t1, true));
}

test "garbage dgrams do not keep the link alive" {
    const t0: Io.Timestamp = .fromNanoseconds(1_000_000_000);
    var link: Link = .{};
    _ = link.update(t0, true);
    for ([_][]const u8{ "", &.{0xFF}, &.{0}, &([_]u8{4} ++ [_]u8{0} ** 13) }, 1..) |garbage, i| {
        const valid = !std.meta.isError(protocol.parseDgram(garbage));
        try std.testing.expectEqual(null, link.update(t0.addDuration(.fromMilliseconds(@intCast(i * 100))), valid));
    }
    try std.testing.expect(link.update(t0.addDuration(connection_timeout).addDuration(.fromNanoseconds(1)), false).? == .became_lost);
}

test "loopback: goes lost within 2x timeout after packets stop, then recovers" {
    const io = std.testing.io;
    var node = initAt(io, .{ .ip4 = .loopback(0) });
    defer node.socket.close(io);
    node.io = io;
    const sender = try (net.IpAddress{ .ip4 = .loopback(0) }).bind(io, .{ .protocol = .udp, .mode = .dgram });
    defer sender.close(io);
    var instructions: MechanismNode.Instructions = .{};
    var input: inputType = .{};
    var output: outputType = .{ .mechanism_instructions = &instructions };
    const packet = [_]u8{@intFromEnum(protocol.ClientMessageType.ending_connection)} ++ [_]u8{0} ** 12;

    for (0..5) |_| {
        try sender.send(io, &node.socket.address, &packet);
        node.update(&input, &output);
    }
    try std.testing.expectEqual(.connected, node.connectionState());

    const stopped = Io.Timestamp.now(io, .awake);
    while (node.connectionState() == .connected and stopped.untilNow(io, .awake).nanoseconds <= 2 * connection_timeout.nanoseconds) node.update(&input, &output);
    try std.testing.expectEqual(.lost, node.connectionState());

    try sender.send(io, &node.socket.address, &packet);
    node.update(&input, &output);
    try std.testing.expectEqual(.connected, node.connectionState());
}
