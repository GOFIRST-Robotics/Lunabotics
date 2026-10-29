// Use candump / cansend

const std = @import("std");
const linux = std.os.linux;
const posix = std.posix;
const linux_can = @import("linux_can.h");
const Io = std.Io;

const packets = @import("packets.zig");

pub const CanId = packed struct(u32) {
    identifier: u29,
    err: bool,
    rtr: bool,
    eff: bool,
};

pub const CanFrame = extern struct {
    id: CanId,
    len: u8,
    _padding: u8 = 0,
    _reserved: u8 = 0,
    len8_dlc: u8 = 0,
    data: [8]u8 align(8) = @splat(0),
};

pub const VescCanId = packed struct(u29) {
    vesc_id: u8,
    packet_type: u8,
    _unused: u13 = 0,

    pub fn fromCanId(id: CanId) ?@This() {
        if (id.err or id.rtr or !id.eff) {
            return null;
        }

        const frame_id: @This() = @bitCast(id.identifier);
        if (frame_id._unused != 0) {
            return null;
        }

        return frame_id;
    }

    pub fn toCanId(self: @This()) CanId {
        return .{
            .identifier = @bitCast(self),
            .err = false,
            .rtr = false,
            .eff = true,
        };
    }
};

pub const BcmCanSocket = struct {
    fd: linux.socket_t,
};

pub fn openSocket(interface_name: ?[]const u8, errno_out: ?*linux.E) !BcmCanSocket {
    if (interface_name) |*n| {
        if (n.len + 1 > linux.IFNAMESIZE) {
            return error.NameTooLong;
        }
    }

    const socket_fd: linux.socket_t = get_sock: {
        const rc = linux.socket(linux.PF.CAN, linux.SOCK.DGRAM, linux_can.CAN_BCM);
        const e = linux.errno(rc);
        if (e != .SUCCESS) {
            if (errno_out) |ptr| {
                ptr.* = e;
            }
            return error.SocketError;
        }
        break :get_sock @intCast(rc);
    };

    const interface_index: c_int = get_index: {
        if (interface_name) |*n| {
            var ifr: linux.ifreq = undefined;
            @memcpy(ifr.ifrn.name[0..n.len], n.ptr);
            ifr.ifrn.name[n.len] = 0;

            const e = linux.errno(linux.ioctl(socket_fd, linux.SIOCGIFINDEX, @intFromPtr(&ifr)));
            if (e != .SUCCESS) {
                if (errno_out) |ptr| {
                    ptr.* = e;
                }
                return error.IoctlError;
            }
            break :get_index ifr.ifru.ivalue;
        } else {
            break :get_index 0;
        }
    };

    {
        const addr: linux_can.sockaddr_can = .{
            .can_family = linux.AF.CAN,
            .can_ifindex = interface_index,
        };

        const e = linux.errno(linux.connect(socket_fd, @ptrCast(&addr), @sizeOf(@TypeOf(addr))));
        if (e != .SUCCESS) {
            if (errno_out) |ptr| {
                ptr.* = e;
            }
            return error.ConnectionError;
        }
    }

    {
        const timestamp_enable: c_int = 1;
        const e = linux.errno(linux.setsockopt(
            socket_fd,
            linux.SOL.SOCKET,
            linux.SO.TIMESTAMPNS_NEW,
            @ptrCast(&timestamp_enable),
            @sizeOf(@TypeOf(timestamp_enable)),
        ));
        if (e != .SUCCESS) {
            if (errno_out) |ptr| {
                ptr.* = e;
            }
            return error.SetSockOptError;
        }
    }

    return .{ .fd = socket_fd };
}

pub fn closeSocket(socket: BcmCanSocket) void {
    _ = linux.close(socket.fd);
}

pub fn sendOnce(io: Io, socket: BcmCanSocket, vesc_id: u8, packet: packets.Packet) !void {
    const frame = try packets.serializeIntoFrame(vesc_id, packet);

    const msg = BcmMessageBuf(1){
        .header = .{
            .opcode = linux_can.TX_SEND,
            .nframes = 1,
            .can_id = @bitCast(frame.id),
        },
        .frames = .{frame},
    };

    try bcmWrite(io, socket.fd, 1, msg);
}

pub const BcmTaskHandle = struct {
    can_id: CanId,
    kind: enum { tx, rx },
};

pub const RepeatingSendOptions = struct {
    vesc_id: u8,
    packet: packets.Packet,
    interval: linux_can.bcm_timeval,
};

pub fn setupRepeatingSend(io: Io, socket: BcmCanSocket, options: RepeatingSendOptions) !BcmTaskHandle {
    const frame = try packets.serializeIntoFrame(options.vesc_id, options.packet);

    const msg = BcmMessageBuf(1){
        .header = .{
            .opcode = linux_can.TX_SETUP,
            .flags = linux_can.SETTIMER | linux_can.STARTTIMER,
            .count = 0,
            .nframes = 1,
            .ival2 = options.interval,
            .can_id = @bitCast(frame.id),
        },
        .frames = .{frame},
    };

    try bcmWrite(io, socket.fd, 1, msg);
    return .{ .can_id = frame.id, .kind = .tx };
}

pub const ReceiveOptions = struct {
    vesc_id: u8,
    packet_type: packets.PacketType,
    ignore_duplicates: bool,
    timeout: ?linux_can.bcm_timeval,
    rate_limit: ?linux_can.bcm_timeval,
};

pub fn setupReceive(io: Io, socket: BcmCanSocket, options: ReceiveOptions) !BcmTaskHandle {
    const can_id = (VescCanId{
        .vesc_id = options.vesc_id,
        .packet_type = @intFromEnum(options.packet_type),
    }).toCanId();

    var msg = BcmMessageBuf(1){
        .header = .{
            .opcode = linux_can.RX_SETUP,
            .flags = 0,
            .can_id = @bitCast(can_id),
        },
        .frames = .{.{
            .id = @bitCast(can_id),
            .len = 8,
            .data = @splat(0xFF),
        }},
    };

    if (options.ignore_duplicates) {
        msg.header.nframes = 1;
    } else {
        msg.header.nframes = 0;
        msg.header.flags |= linux_can.RX_FILTER_ID;
    }

    if (options.timeout) |timeout| {
        msg.header.ival1 = timeout;
        msg.header.flags |= linux_can.SETTIMER | linux_can.STARTTIMER | linux_can.RX_ANNOUNCE_RESUME;
    } else {
        msg.header.ival1 = .{ .tv_sec = 0, .tv_usec = 0 };
    }

    if (options.rate_limit) |rate_limit| {
        msg.header.ival2 = rate_limit;
        msg.header.flags |= linux_can.SETTIMER;
    } else {
        msg.header.ival2 = .{ .tv_sec = 0, .tv_usec = 0 };
    }

    try bcmWrite(io, socket.fd, 1, msg);
    return .{ .can_id = can_id, .kind = .rx };
}

pub fn deleteTask(io: Io, socket: BcmCanSocket, handle: BcmTaskHandle) !void {
    const msg = BcmMessageBuf(0){
        .header = .{
            .opcode = switch (handle.kind) {
                .tx => linux_can.TX_DELETE,
                .rx => linux_can.RX_DELETE,
            },
            .flags = 0,
            .can_id = handle.can_id,
            .nframes = 0,
        },
        .frames = .{},
    };

    try bcmWrite(io, socket.fd, 0, msg);
}

pub const VescEventData = union(enum) {
    packet_received: packets.Packet,
    packet_timeout: packets.PacketType,
};

pub const VescEvent = struct {
    vesc_id: u8,
    timestamp: linux.kernel_timespec,
    data: VescEventData,
};

// Blocks until next event is ready
pub fn readNextEvent(io: Io, socket: BcmCanSocket) !VescEvent {
    // Keep reading and processing BCM messages until something interesting happens,
    // in which case we return it as an event
    while (true) {
        const bcm_buffer, const timestamp = try bcmRead(io, socket.fd, 1);

        switch (bcm_buffer.header.opcode) {
            linux_can.RX_CHANGED => {
                // New CAN data incoming
                if (bcm_buffer.header.nframes < 1) {
                    return error.MissingFrame;
                }

                const can_frame = bcm_buffer.frames[0];
                const packet_from_vesc = try packets.parseFrame(can_frame);
                if (packet_from_vesc) |p| {
                    return .{
                        .vesc_id = p.source_vesc_id,
                        .timestamp = timestamp,
                        .data = .{ .packet_received = p.packet },
                    };
                }
            },
            linux_can.RX_TIMEOUT => {
                if (VescCanId.fromCanId(@bitCast(bcm_buffer.header.can_id))) |vesc_can_id| {
                    // Device is die
                    if (std.enums.fromInt(packets.PacketType, vesc_can_id.packet_type)) |packet_type| {
                        return .{
                            .vesc_id = vesc_can_id.vesc_id,
                            .timestamp = timestamp,
                            .data = .{ .packet_timeout = packet_type },
                        };
                    }
                }
            },
            else => {},
        }
    }
}

fn BcmMessageBuf(max_frames: usize) type {
    return extern struct {
        header: linux_can.bcm_msg_head,
        frames: [max_frames]CanFrame,
    };
}

fn bcmRead(
    io: Io,
    fd: linux.socket_t,
    comptime max_frames: usize,
) !struct { BcmMessageBuf(max_frames), linux.kernel_timespec } {
    // Will hold the BCM event data and CAN frames
    var bcm_buffer: BcmMessageBuf(max_frames) = undefined;

    // Will hold the timestamp of the BCM event message (at most one)
    const cmsg_header_size = comptime @sizeOf(linux.cmsghdr);
    const cmsg_data_offset = comptime std.mem.alignForward(usize, cmsg_header_size, @alignOf(usize)); // Alignment padding
    const cmsg_buffer_size = comptime std.mem.alignForward(usize, cmsg_data_offset + @sizeOf(linux.kernel_timespec), @alignOf(usize));
    var cmsg_buffer: [cmsg_buffer_size]u8 align(@alignOf(linux.cmsghdr)) = undefined;

    var message_buffer = [_]Io.net.IncomingMessage{.{
        .data = undefined,
        .from = undefined,
        .control = std.mem.asBytes(&cmsg_buffer),
        .flags = undefined,
    }};

    // .net_receive operation maps to recvmsg() syscall, which will read at most one BCM message
    const op_result = try io.operate(.{
        .net_receive = .{
            .socket_handle = fd,
            .message_buffer = &message_buffer,
            .data_buffer = std.mem.asBytes(&bcm_buffer),
            .flags = .{},
        },
    });

    const recv_err, const recv_count = op_result.net_receive;
    if (recv_err) |err| return err;
    if (recv_count == 0) return error.NoData;

    const received_msg = message_buffer[0];
    if (received_msg.data.len < @sizeOf(linux_can.bcm_msg_head)) {
        // BCM header is incomplete
        return error.MessageTooSmall;
    }
    if (received_msg.data.len < @sizeOf(linux_can.bcm_msg_head) + bcm_buffer.header.nframes * @sizeOf(CanFrame)) {
        // Did not receive all the frames we were supposed to
        return error.MessageTooSmall;
    }
    // Now we know the BCM buffer is good

    if (received_msg.control.len < 1) {
        return error.MissingTimestamp;
    }

    // Since the only CMSG we enabled is timestamping, we know the one message we received is the timestamp
    const cmsg: *const linux.cmsghdr = @ptrCast(&cmsg_buffer);
    const is_timestamp = cmsg.type == linux.SO.TIMESTAMPNS_OLD or cmsg.type == linux.SO.TIMESTAMPNS_NEW;
    if (cmsg.level != linux.SOL.SOCKET or !is_timestamp) {
        std.debug.print("{any}", .{cmsg.type});
        return error.UnexpectedCmsg;
    }
    const timestamp_ptr: *const linux.kernel_timespec = @ptrCast(@alignCast(&cmsg_buffer[cmsg_data_offset]));

    return .{ bcm_buffer, timestamp_ptr.* };
}

fn bcmWrite(io: Io, fd: linux.socket_t, comptime max_frames: usize, buf: BcmMessageBuf(max_frames)) !void {
    std.debug.assert(buf.header.nframes <= max_frames);
    const write_size = @sizeOf(linux_can.bcm_msg_head) + buf.header.nframes * @sizeOf(CanFrame);
    _ = try io.operate(.{
        .file_write_streaming = .{
            .file = .{ .handle = fd, .flags = .{ .nonblocking = false } },
            .data = &.{
                std.mem.asBytes(&buf)[0..write_size],
            },
        },
    });
}
