const std = @import("std");
const linux = std.os.linux;
const linux_can = @cImport({
    @cInclude("linux/can.h");
    @cInclude("linux/can/bcm.h");
});
const Allocator = std.mem.Allocator;

const vesc_commands = @import("vesc_commands.zig");
const vesc_packets = @import("vesc_packets.zig");
const PacketType = vesc_packets.PacketType;

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

pub const VescEvent = union(enum) {
    notify_boot: vesc_packets.NotifyBoot,
    command: vesc_commands.Command,
};

pub const VescConfig = struct {
    vesc_id: u8,
};

pub const VescBusConfig = struct {
    interface_name: ?[]const u8,
    vesc_configs: []const VescConfig,
};

pub const VescBus = struct {
    fd: linux.socket_t,
    read_buffer: *BcmReadBuffer(1),
    read_index: usize,
    read_count: usize,

    allocator: Allocator,

    pub const OpenError = BcmCanSocketOpenError || std.mem.Allocator.Error || FilterSetupError;
    pub fn open(allocator: Allocator, config: VescBusConfig, errno_out: ?*linux.E) OpenError!@This() {
        const fd = try openBcmCanSocket(config.name, errno_out);

        // TODO: Put a timeout listener on one of these to monitor if device is connected
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
                try setupStatusInputFilter(fd, vesc_config.vesc_id, status, errno_out);
            }
            for (events) |event| {
                try setupEventInputFilter(fd, vesc_config.vesc_id, event, errno_out);
            }
        }

        const read_buffer = try allocator.create(BcmReadBuffer(1));
        read_buffer.initInPlace();

        return .{
            .fd = fd,
            .read_buffer = read_buffer,
            .read_index = 0,
            .read_count = 0,
            .allocator = allocator,
        };
    }

    fn readIfBufferEmpty(self: *@This(), errno_out: ?*linux.E) BcmReadError!void {
        if (self.read_index >= self.read_count) {
            const message_count = try bcmRead(self.fd, 1, &self.read_buffer, errno_out);
            self.read_index = 0;
            self.read_count = message_count;
        }
    }

    fn processPacket(self: *@This(), vesc_id: u8, packet: vesc_packets.Packet) ?VescEvent {
        _ = self;
        std.debug.print("Received packet from {d}: {any}\n", .{ vesc_id, packet });
        return null;
    }

    pub const PollEventsError = BcmReadError || vesc_packets.ParseError || error{IncompleteMessage};
    pub fn pollEvents(self: *@This(), errno_out: ?*linux.E) PollEventsError!?VescEvent {
        try self.readIfBufferEmpty(errno_out);

        while (self.read_index < self.read_count) {
            // Must always increment index regardless of if an error occurs
            const index = self.read_index;
            self.read_index += 1;

            // Ensure we actually got all the data
            const message_len = self.read_buffer.msg_headers[index].len;
            if (message_len < @sizeOf(BcmMessageBuf(0))) {
                return error.IncompleteMessage;
            }
            const frame_count = self.read_buffer.rx_buffers[index].header.nframes;
            if (message_len < @sizeOf(BcmMessageBuf(0)) + frame_count * @sizeOf(CanFrame)) {
                return error.IncompleteMessage;
            }
            if (frame_count == 0) {
                // Message had no frame for some reason, ignore
                continue;
            }

            const can_frame = self.read_buffer.rx_buffers[index].frames[0];
            const packet_from_vesc = try vesc_packets.parseFrame(can_frame);
            if (packet_from_vesc) |p| {
                if (self.processPacket(p.source_vesc_id, p.packet)) |event| {
                    return event;
                }
            }

            // Refill read buffer with incoming data
            try self.readIfBufferEmpty(errno_out);
        }

        // Processed all incoming data, no event occurred
        return null;
    }

    pub fn close(self: @This()) void {
        _ = linux.close(self.fd);
        self.allocator.destroy(self.read_buffer);
    }
};

const BcmCanSocketOpenError = error{ NameTooLong, SocketError, IoctlError, ConnectionError };
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

    return socket_fd;
}

fn BcmMessageBuf(max_frames: usize) type {
    return extern struct {
        header: linux_can.bcm_msg_head,
        frames: [max_frames]CanFrame,
    };
}

fn BcmReadBuffer(max_frames: usize) type {
    const max_burst = linux_can.MAX_NFRAMES;

    return struct {
        rx_buffers: [max_burst]BcmMessageBuf(max_frames),
        iovecs: [max_burst]linux.iovec,
        msg_headers: [max_burst]linux.mmsghdr,

        pub fn initInPlace(self: *@This()) void {
            for (0..max_burst) |i| {
                self.iovecs[i] = .{
                    .base = @ptrCast(&self.rx_buffers[i]),
                    .len = @sizeOf(BcmMessageBuf(max_frames)),
                };

                self.msg_headers[i] = .{
                    .hdr = .{
                        .name = null,
                        .namelen = 0,
                        .iov = @ptrCast(&self.iovecs[i]),
                        .iovlen = 1,
                        .control = null,
                        .controllen = 0,
                        .flags = 0,
                    },
                    .len = 0,
                };
            }
        }
    };
}

// FIXME: Figure out making this non-blocking
const BcmReadError = error{ReadFailed};
fn bcmRead(fd: linux.socket_t, comptime max_frames: usize, buf: *BcmReadBuffer(max_frames), errno_out: ?*linux.E) BcmReadError!usize {
    const rc = linux.recvmmsg(fd, &buf.msg_headers, buf.msg_headers.len, 0, null);
    const e = linux.errno(rc);
    if (e != .SUCCESS) {
        if (errno_out) |ptr| {
            ptr.* = e;
        }
        return error.ReadFailed;
    }
    return @intCast(rc);
}

const BcmWriteError = error{WriteFailed};
fn bcmWrite(fd: linux.socket_t, comptime max_frames: usize, buf: BcmMessageBuf(max_frames), errno_out: ?*linux.E) BcmWriteError!void {
    const write_size = @sizeOf(linux_can.bcm_msg_head) + buf.nframes * @sizeOf(CanFrame);
    const ret = linux.errno(linux.write(fd, std.mem.asBytes(&buf), write_size));
    if (ret != .SUCCESS) {
        if (errno_out) |e| {
            e.* = ret;
        }
        return error.WriteFailed;
    }
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

const FilterSetupError = error{FilterSetupFailed};
fn setupStatusInputFilter(fd: linux.socket_t, vesc_id: u8, packet_type: PacketType, errno_out: ?*linux.E) FilterSetupError!void {
    const frame_id = (VescCanId{
        .vesc_id = vesc_id,
        .packet_type = @intFromEnum(packet_type),
    }).toCanId();

    // Configure to only process changes in data
    const msg = BcmMessageBuf(1){
        .header = .{
            .opcode = linux_can.RX_SETUP,
            .can_id = @bitCast(frame_id),
            .nframes = 1,
        },
        .frames = .{.{
            .id = @bitCast(frame_id),
            .len = 8,
            .data = @splat(0xFF),
        }},
    };

    bcmWrite(fd, 1, msg, errno_out) catch return error.FilterSetupError;
}

fn setupEventInputFilter(fd: linux.socket_t, vesc_id: u8, packet_type: PacketType, errno_out: ?*linux.E) FilterSetupError!void {
    const frame_id = (VescCanId{
        .vesc_id = vesc_id,
        .packet_type = @intFromEnum(packet_type),
    }).toCanId();

    // Configure to receive all packets with the ID, regardless of data
    const msg = BcmMessageBuf(0){
        .header = .{
            .opcode = linux_can.RX_SETUP,
            .flags = linux_can.RX_FILTER_ID,
            .can_id = @bitCast(frame_id),
            .nframes = 0,
        },
        .frames = .{},
    };

    bcmWrite(fd, 1, msg, errno_out) catch return error.FilterSetupError;
}
