const std = @import("std");
const linux = std.os.linux;
const protocol = @import("protocol.zig");
const Io = std.Io;
const net = Io.net;
const MechanismNode = @import("../robot_logic/MechanismNode.zig");

const ServerNode = @This();

const logger = std.log.scoped(.server);

const server_address: net.IpAddress = .{ .ip4 = .{ .port = protocol.port, .bytes = .{ 0, 0, 0, 0 } } };

threaded: Io.Threaded,
io: Io,
socket: net.Socket,

previous_controller_data: protocol.ControllerData,
last_controller_time: protocol.TimeStamp,
controller_up: bool,

previous_stream_deck_data: protocol.StreamDeckData,
last_stream_deck_time: protocol.TimeStamp,
stream_deck_up: bool,

pub const inputType = struct {};

pub const outputType = struct {
    mechanism_instructions: *MechanismNode.Instructions,
    pub fn linkBuffer(self: *@This(), mech: *MechanismNode.inputType) void {
        self.mechanism_instructions = &mech.instructions;
    }
};

pub fn init() @This() {
    var ret: @This() = undefined;
    ret.threaded = .init_single_threaded;
    ret.io = ret.threaded.io();

    ret.previous_controller_data = .{};
    ret.previous_stream_deck_data = .{};
    ret.controller_up = false;
    ret.stream_deck_up = false;

    // There are no obvious defaults for these
    // but logic wise it makes sense to just assume the last data was too long ago
    ret.last_controller_time = .{ .nano_seconds = 0, .seconds = 0 };
    ret.last_stream_deck_time = .{ .nano_seconds = 0, .seconds = 0 };

    ret.socket = server_address.bind(ret.io, .{ .protocol = .udp, .mode = .dgram }) catch @panic("Failed to open UDP controller connection");
    logger.info("Successfully bound server to port {d}", .{protocol.port});

    return ret;
}

pub fn deinit(self: *@This()) void {
    _ = self;
}
pub fn update(self: *@This(), input: *inputType, output: *outputType) void {
    _ = input;

    const current_time: protocol.TimeStamp = gettime: {
        var timespec: linux.timespec = undefined;
        const rc = linux.errno(linux.clock_gettime(.MONOTONIC, &timespec));
        if (rc != .SUCCESS) {
            logger.warn("Faield to get current time with clock_gettime; Errno: {s}", .{@tagName(rc)});
            return;
        }
        break :gettime .fromTimeSpec(&timespec);
    };

    var dgram_buf: [1024]u8 = undefined;

    const dgram_info = self.socket.receive(self.io, &dgram_buf) catch |err| {
        logger.warn("Failed to read socket, {s}", .{@errorName(err)});
        return;
    };
    const dgram_data = dgram_info.data;
    var dgram_index: usize = 0;

    const message_type = protocol.takeFromDgram(protocol.ClientMessageType, dgram_data);
    dgram_index += @sizeOf(protocol.ClientMessageType);
    const message_time = protocol.takeFromDgram(protocol.TimeStamp, dgram_data[dgram_index..]);
    dgram_index += @sizeOf(protocol.TimeStamp);

    if (protocol.isOlderBy(current_time, message_time, 1, 0)) {
        logger.info("Got stale data\ncurrent time: {d}:{d}\nmessage time: {d}:{d}", .{
            current_time.seconds,
            current_time.nano_seconds,
            message_time.seconds,
            message_time.nano_seconds,
        });
        return;
    }

    switch (message_type) {
        .controller => {
            // if this is true then this dgram is either old data or a duplicate
            if (protocol.isOlder(message_time, self.last_controller_time)) {
                return;
            }

            const controller_data = protocol.takeFromDgram(protocol.ControllerData, dgram_data[dgram_index..]);
            dgram_index += @sizeOf(protocol.ControllerData);

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
        .stream_deck => {
            // if this is true then this dgram is either old data or a duplicate
            if (protocol.isOlder(message_time, self.last_stream_deck_time)) {
                return;
            }

            const stream_deck_data = protocol.takeFromDgram(protocol.StreamDeckData, dgram_data[dgram_index..]);
            dgram_index += @sizeOf(protocol.StreamDeckData);

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
        else => {
            return;
        },
    }

    // if this is not true then some data has not been parsed which is (most likely) an error
    std.debug.assert(dgram_data.len == dgram_index);

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
