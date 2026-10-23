// Use candump / cansend

const std = @import("std");
const linux = std.os.linux;
const posix = std.posix;
const linux_can = @cImport({
    @cInclude("linux/can.h");
    @cInclude("linux/can/bcm.h");
    @cInclude("linux/net_tstamp.h");
});
const Io = std.Io;
const Allocator = std.mem.Allocator;

const packets = @import("packets.zig");
const PacketType = packets.PacketType;
const commands = @import("commands.zig");

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

pub const VescEventData = union(enum) {
    // Triggered only if the status data has changed, or a status packet is received
    // after previously timing out
    status_1: packets.Status1,
    status_2: packets.Status2,
    status_3: packets.Status3,
    status_4: packets.Status4,
    status_5: packets.Status5,
    status_6: packets.Status6,

    notify_boot: packets.NotifyBoot,
    command: commands.Command,

    // Triggered if a status packet has not been received for too long
    timeout,
};

pub const VescEvent = struct {
    vesc_id: u8,
    timestamp: linux.kernel_timespec,
    data: VescEventData,
};

pub const VescConfig = struct {
    vesc_id: u8,
};

pub const VescBusConfig = struct {
    interface_name: ?[]const u8,
    vesc_configs: []const VescConfig,
};

const RX_BUFFER_LEN: usize = 512;

const VescState = struct {
    vesc_id: u8,
    rx_buffer: [RX_BUFFER_LEN]u8,
};

pub const VescBus = struct {
    fd: linux.socket_t,
    vesc_states: []VescState,
    allocator: Allocator,

    pub fn open(io: Io, allocator: Allocator, config: VescBusConfig, errno_out: ?*linux.E) !@This() {
        const fd = try openBcmCanSocket(config.interface_name, errno_out);

        const statuses = [_]PacketType{
            .status_1,
            .status_2,
            .status_3,
            .status_4,
            .status_5,
            .status_6,
        };
        const events = [_]PacketType{
            .notify_boot,
            .fill_rx_buffer,
            .fill_rx_buffer_long,
            .process_rx_buffer,
            .process_short_buffer,
        };

        for (config.vesc_configs) |vesc_config| {
            for (statuses) |status| {
                try setupInputFilter(io, fd, .{
                    .vesc_id = vesc_config.vesc_id,
                    .packet_type = status,
                    .ignore_duplicates = true,
                    // TODO: We probably want these for status signals
                    .timeout = .{ .tv_sec = 1, .tv_usec = 0 },
                    .rate_limit = null,
                });
            }
            for (events) |event| {
                try setupInputFilter(io, fd, .{
                    .vesc_id = vesc_config.vesc_id,
                    .packet_type = event,
                    .ignore_duplicates = false,
                    .timeout = null,
                    .rate_limit = null,
                });
            }
        }

        const vesc_states = try allocator.alloc(VescState, config.vesc_configs.len);
        for (0..vesc_states.len) |i| {
            vesc_states[i].vesc_id = config.vesc_configs[i].vesc_id;
        }

        return .{
            .fd = fd,
            .vesc_states = vesc_states,
            .allocator = allocator,
        };
    }

    fn findVescState(self: *const @This(), vesc_id: u8) ?*VescState {
        for (0..self.vesc_states.len) |i| {
            if (self.vesc_states[i].vesc_id == vesc_id) {
                return &self.vesc_states[i];
            }
        }
        return null;
    }

    fn processPacket(self: *@This(), vesc_id: u8, packet: packets.Packet) ?VescEventData {
        _ = self; // Will need eventually when implementing the multi-packet commands

        switch (packet) {
            .status_1 => |p| return .{ .status_1 = p },
            .status_2 => |p| return .{ .status_2 = p },
            .status_3 => |p| return .{ .status_3 = p },
            .status_4 => |p| return .{ .status_4 = p },
            .status_5 => |p| return .{ .status_5 = p },
            .status_6 => |p| return .{ .status_6 = p },
            .notify_boot => |p| return .{ .notify_boot = p },

            else => {
                std.debug.print("Received unhandled packet from {d}: {any}\n", .{ vesc_id, packet });
                return null;
            },
        }
    }

    pub fn readNextEvent(self: *@This(), io: Io) !VescEvent {
        // Keep reading and processing BCM messages until something interesting happens,
        // in which case we return it as an event
        while (true) {
            const bcm_buffer, const timestamp = try bcmRead(io, self.fd, 1);

            switch (bcm_buffer.header.opcode) {
                linux_can.RX_CHANGED => {
                    // New CAN data incoming
                    if (bcm_buffer.header.nframes < 1) {
                        return error.MissingFrame;
                    }

                    const can_frame = bcm_buffer.frames[0];
                    const packet_from_vesc = try packets.parseFrame(can_frame);
                    if (packet_from_vesc) |p| {
                        if (self.processPacket(p.source_vesc_id, p.packet)) |event_data| {
                            return .{
                                .vesc_id = p.source_vesc_id,
                                .timestamp = timestamp,
                                .data = event_data,
                            };
                        }
                    }
                },
                linux_can.RX_TIMEOUT => {
                    if (VescCanId.fromCanId(@bitCast(bcm_buffer.header.can_id))) |vesc_can_id| {
                        // Device is die
                        return .{
                            .vesc_id = vesc_can_id.vesc_id,
                            .timestamp = timestamp,
                            .data = .timeout,
                        };
                    }
                },
                else => {},
            }
        }
    }

    pub fn close(self: @This()) void {
        _ = linux.close(self.fd);
        self.allocator.free(self.vesc_states);
    }
};

const BcmCanSocketOpenError = error{ NameTooLong, SocketError, IoctlError, ConnectionError, SetSockOptError };
fn openBcmCanSocket(name: ?[]const u8, errno_out: ?*linux.E) BcmCanSocketOpenError!linux.socket_t {
    if (name) |*n| {
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
        if (name) |*n| {
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

    return socket_fd;
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

const FilterOptions = struct {
    vesc_id: u8,
    packet_type: PacketType,
    ignore_duplicates: bool,
    timeout: ?linux_can.bcm_timeval,
    rate_limit: ?linux_can.bcm_timeval,
};

fn setupInputFilter(io: Io, fd: linux.socket_t, options: FilterOptions) !void {
    const frame_id = (VescCanId{
        .vesc_id = options.vesc_id,
        .packet_type = @intFromEnum(options.packet_type),
    }).toCanId();

    var msg = BcmMessageBuf(1){
        .header = .{
            .opcode = linux_can.RX_SETUP,
            .flags = 0,
            .can_id = @bitCast(frame_id),
        },
        .frames = .{.{
            .id = @bitCast(frame_id),
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

    try bcmWrite(io, fd, 1, msg);
}
