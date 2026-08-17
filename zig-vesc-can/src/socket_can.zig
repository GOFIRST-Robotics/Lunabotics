const std = @import("std");
const Io = std.Io;
const net = Io.net;
const linux = std.os.linux;
const vesc_datatypes = @import("vesc_datatypes.zig");
const can = @import("can.h");
const vescWrite = vesc_datatypes.vescWrite;
const simulate_motor = @import("simulate_motor.zig");
const bytesAsValue = std.mem.bytesAsValue;

pub const _can_frame = can.can_frame;

const CommandType = vesc_datatypes.CommandType;

pub const Id = packed struct(u32) {
    vesc_id: u8,
    command_type: CommandType,
    _unused: u13 = 0,
    err: bool = false,
    rtr: bool = false,
    eff: bool = true, // this must be true
};

const DataTypes = extern union {
    bytes: [8]u8 align(1),
    set_current: vesc_datatypes.SetCurrent align(1),
    set_duty: vesc_datatypes.SetDuty align(1),
    set_rpm: vesc_datatypes.SetRPM align(1),
    set_pos: vesc_datatypes.SetPos align(1),
};

pub const CanFrame = extern struct {
    id: Id,
    len: u8,
    _pad: u8 = 0, // do not use
    _res0: u8 = 0, // do not use
    _len8_dlc: u8 = 0, // VESC does not use this
    data: DataTypes align(8) = .{ .bytes = @splat(0) },

    pub fn createSetDuty(vesc_id: u8, duty: f32) @This() {
        return .{
            .id = .{ .vesc_id = vesc_id, .command_type = .SET_DUTY },
            .len = 4,
            .data = .{ .set_duty = .{ .duty = vescWrite(u32, duty, 100_000) } },
        };
    }

    pub fn createSetCurrent(vesc_id: u8, current: f32) @This() {
        return .{
            .id = .{ .vesc_id = vesc_id, .command_type = .SET_CURRENT },
            .len = 4,
            .data = .{ .set_current = .{ .current = vescWrite(i32, current, 1_000) } },
        };
    }

    pub fn createSetRPM(vesc_id: u8, rpm: f32) @This() {
        return .{
            .id = .{ .vesc_id = vesc_id, .command_type = .SET_RPM },
            .len = 4,
            .data = .{ .set_current = .{ .current = vescWrite(u32, rpm, 1) } },
        };
    }

    pub fn createSetPos(vesc_id: u8, pos: f32) @This() {
        return .{
            .id = .{ .vesc_id = vesc_id, .command_type = .SET_POS },
            .len = 4,
            .data = .{ .set_pos = .{ .pos = vescWrite(u32, pos, 1_000_000) } },
        };
    }
};

pub const CanFilter = can.can_filter;

pub const VescFilter = struct {
    vesc_id: ?u8 = null,
    command_type: ?CommandType = null,
};

/// if name is null then the returned socket will bind to all CAN networks
/// if an error occures the errno will be set, otherwise it will be left unchanged
pub const RawSocketCANError = error{ SocketError, IoctlError, BindError };
pub fn openCANRaw(name: ?[]const u8, errno: ?*linux.E) RawSocketCANError!linux.socket_t {
    const socket_fd: linux.socket_t = get_sock: {
        const rc = linux.socket(linux.PF.CAN, linux.SOCK.RAW, can.CAN_RAW);
        const e: linux.E = linux.errno(rc);
        if (e != .SUCCESS) {
            if (errno) |ptr| {
                ptr.* = e;
            }
            return RawSocketCANError.SocketError;
        }
        break :get_sock @intCast(rc);
    };

    const ifindex: c_int = get_index: {
        if (name) |*n| {
            var ifr: linux.ifreq = undefined;
            @memcpy(ifr.ifrn.name[0..n.len], n.ptr);
            ifr.ifrn.name[n.len] = 0;
            const ioctl_rc = linux.errno(linux.ioctl(socket_fd, linux.SIOCGIFINDEX, @intFromPtr(&ifr)));
            if (ioctl_rc != .SUCCESS) {
                if (errno) |ptr| {
                    ptr.* = ioctl_rc;
                }
                return RawSocketCANError.IoctlError;
            }
            break :get_index ifr.ifru.ivalue;
        } else {
            break :get_index 0;
        }
    };

    const addr: can.sockaddr_can = .{ .can_family = linux.AF.CAN, .can_ifindex = ifindex };

    const bind_rc = linux.errno(linux.bind(socket_fd, @ptrCast(&addr), @sizeOf(@TypeOf(addr))));
    if (bind_rc != .SUCCESS) {
        if (errno) |ptr| {
            ptr.* = bind_rc;
        }
        return RawSocketCANError.BindError;
    }

    return socket_fd;
}

pub const BCMSocketCANError = error{
    SocketError,
    IoctlError,
    ConnectionError,
    SetSockOptError,
};
pub fn openCANBCM(name: ?[]const u8, errno: ?*linux.E) BCMSocketCANError!linux.socket_t {
    const socket_fd: linux.socket_t = get_sock: {
        const rc = linux.socket(linux.PF.CAN, linux.SOCK.DGRAM, can.CAN_BCM);
        const e: linux.E = linux.errno(rc);
        if (e != .SUCCESS) {
            if (errno) |ptr| {
                ptr.* = e;
            }
            return BCMSocketCANError.SocketError;
        }
        break :get_sock @intCast(rc);
    };

    const ifindex: c_int = get_index: {
        if (name) |*n| {
            var ifr: linux.ifreq = undefined;
            @memcpy(ifr.ifrn.name[0..n.len], n.ptr);
            ifr.ifrn.name[n.len] = 0;
            const ioctl_rc = linux.errno(linux.ioctl(socket_fd, linux.SIOCGIFINDEX, @intFromPtr(&ifr)));
            if (ioctl_rc != .SUCCESS) {
                if (errno) |ptr| {
                    ptr.* = ioctl_rc;
                }
                return BCMSocketCANError.IoctlError;
            }
            break :get_index ifr.ifru.ivalue;
        } else {
            break :get_index 0;
        }
    };

    const addr: can.sockaddr_can = .{ .can_family = linux.AF.CAN, .can_ifindex = ifindex };

    const connect_rc = linux.errno(linux.connect(socket_fd, @ptrCast(&addr), @sizeOf(@TypeOf(addr))));
    if (connect_rc != .SUCCESS) {
        if (errno) |ptr| {
            ptr.* = connect_rc;
        }
        return BCMSocketCANError.ConnectionError;
    }

    const sock_opts_flags: c_int = can.SOF_TIMESTAMPING_RX_SOFTWARE | can.SOF_TIMESTAMPING_SOFTWARE;
    const set_sock_ops_errno = linux.errno(linux.setsockopt(
        socket_fd,
        linux.SOL.SOCKET,
        linux.SO.TIMESTAMPING_NEW,
        @ptrCast(&sock_opts_flags),
        @sizeOf(@TypeOf(sock_opts_flags)),
    ));

    if (set_sock_ops_errno != .SUCCESS) {
        if (errno) |ptr| {
            ptr.* = set_sock_ops_errno;
        }
        return BCMSocketCANError.SetSockOptError;
    }

    return socket_fd;
}

pub fn sendRawCANFrame(io: Io, socket: linux.socket_t, can_frame: *const CanFrame) !void {
    _ = try io.operate(.{
        .file_write_streaming = .{
            .file = .{ .handle = socket, .flags = .{ .nonblocking = false } },
            .data = &.{
                @ptrCast(can_frame),
            },
        },
    });
}

pub fn sendRawCANFrames(io: Io, socket: linux.socket_t, can_frames: []const CanFrame) !void {
    for (can_frames) |frame| {
        try sendRawCANFrame(io, socket, &frame);
    }
}

pub fn readRawCANFrame(io: Io, socket: linux.socket_t) !CanFrame {
    var ret: CanFrame = undefined;
    _ = try io.operate(.{ .file_read_streaming = .{
        .file = .{ .handle = socket, .flags = .{ .nonblocking = false } },
        .data = &.{@ptrCast(&ret)},
    } });
    return ret;
}

pub fn BCMBuffer(max_frames: usize) type {
    return extern struct {
        msg_head: can.bcm_msg_head,
        can_frames: [max_frames]CanFrame,
    };
}

pub const ReadBCMError = Io.File.ReadStreamingError || Io.Cancelable;
pub fn readBCMCANFrame(
    io: Io,
    socket: linux.socket_t,
    comptime max_frames: usize,
) ReadBCMError!BCMBuffer(max_frames) {
    var ret: BCMBuffer(max_frames) = undefined;

    _ = try io.operate(.{ .file_read_streaming = .{
        .file = .{ .handle = socket, .flags = .{ .nonblocking = false } },
        .data = &.{@ptrCast(&ret)},
    } });
    return ret;
}

const scm_timestamping64 = extern struct {
    ts: [3]linux.kernel_timespec,
};

pub fn recvmesgBCM(
    io: Io,
    can_socket: linux.socket_t,
    comptime max_frames: usize,
) !struct { BCMBuffer(max_frames), Io.net.IncomingMessage } {
    var ret: BCMBuffer(max_frames) = undefined;
    var ctrl_buf: [can.CMSG_SPACE(@sizeOf(scm_timestamping64))]u8 = undefined;
    var message: [1]Io.net.IncomingMessage = .{.init};
    message[0].control = &ctrl_buf;

    const len = try io.operate(.{ .net_receive = .{
        .flags = .{},
        .data_buffer = @ptrCast(&ret),
        .message_buffer = &message,
        .socket_handle = can_socket,
    } });
    _ = len;

    const time_stamp: *scm_timestamping64 = bytesAsValue(
        scm_timestamping64,
        &message[0],
    );
    _ = time_stamp;

    return .{ ret, message[0] };
}

pub fn recvmesgBCM2(
    io: Io,
    can_socket: linux.socket_t,
    comptime max_frames: usize,
) !struct { BCMBuffer(max_frames), linux.kernel_timespec } {
    var data_buf: [256 * 2]u8 align(@alignOf(BCMBuffer(max_frames))) = undefined;
    var ctrl_buf: [256]u8 align(@alignOf(linux.cmsghdr)) = undefined;

    var msg_buf = [_]Io.net.IncomingMessage{.{
        .data = undefined,
        .from = undefined,
        .control = &ctrl_buf,
        .flags = undefined,
    }};

    const op_result = try io.operate(.{
        .net_receive = .{
            .socket_handle = can_socket,
            .message_buffer = &msg_buf,
            .data_buffer = &data_buf,
            .flags = .{},
        },
    });

    const recv_err, const recv_count = op_result.net_receive;
    if (recv_err) |err| return err;
    if (recv_count == 0) return error.NoData;

    const received_msg = msg_buf[0];

    if (received_msg.data.len < @sizeOf(can.bcm_msg_head)) return error.MessageTooSmall;
    const rx_msg = @as(*align(1) const BCMBuffer(max_frames), @ptrCast(received_msg.data.ptr)).*;

    var timestamp: linux.kernel_timespec = undefined;

    const control = received_msg.control;
    var offset: usize = 0;

    while (offset + @sizeOf(linux.cmsghdr) <= control.len) {
        const cmsg = @as(*align(1) const linux.cmsghdr, @ptrCast(control[offset..].ptr));
        if (cmsg.len < @sizeOf(linux.cmsghdr)) break; // Malformed header, exit parsing

        if (cmsg.level == linux.SOL.SOCKET and cmsg.type == linux.SO.TIMESTAMPING_NEW) {
            const data_offset = std.mem.alignForward(usize, offset + @sizeOf(linux.cmsghdr), @alignOf(usize));

            if (data_offset + @sizeOf(scm_timestamping64) <= control.len) {
                const ts_data = @as(*align(1) const scm_timestamping64, @ptrCast(control[data_offset..].ptr));
                timestamp = ts_data.ts[0];
                break;
            }
        }

        // Advance the offset to the next message, aligned to the system's size_t boundary
        offset = std.mem.alignForward(usize, offset + cmsg.len, @alignOf(usize));
    }

    return .{ rx_msg, timestamp };
}

const RawFilterError = error{ConfigError};
/// This command will overide previous filters
pub fn addRawFilters(socket: linux.socket_t, filters: []const CanFilter, errno: ?*linux.E) RawFilterError!void {
    const ret = linux.setsockopt(socket, can.SOL_CAN_RAW, can.CAN_RAW_FILTER, @ptrCast(filters.ptr), @intCast(filters.len * @sizeOf(can.can_filter)));
    if (linux.errno(ret) != .SUCCESS) {
        if (errno) |e| {
            e.* = linux.errno(ret);
        }
        return RawFilterError.ConfigError;
    }
}

pub fn vescFiltersToRawFilters(comptime N: usize, comptime filters: [N]VescFilter) [N]CanFilter {
    const _Id = packed struct(u32) {
        vesc_id: u8 = 0,
        command_type: u8 = 0,
        _unused: u13 = 0,
        err: bool = false,
        rtr: bool = false,
        eff: bool = true,
    };

    const raw_filters: [N]CanFilter = parse_filters: {
        var tmp: [N]CanFilter = undefined;
        for (filters, 0..) |filter, i| {
            const raw_filter: CanFilter = parse_filter: {
                var mask: _Id = .{};
                var match: _Id = .{};
                if (filter.vesc_id) |id| {
                    mask.vesc_id = 0xff;
                    match.vesc_id = id;
                }
                if (filter.command_type) |cmd| {
                    mask.command_type = 0xff;
                    match.command_type = @intFromEnum(cmd);
                }
                // std.debug.print("id: {x}\nmk: {x}\n", .{ @as(u32, @bitCast(match)), @as(u32, @bitCast(mask)) });
                break :parse_filter CanFilter{ .can_id = @bitCast(match), .can_mask = @bitCast(mask) };
            };
            tmp[i] = raw_filter;
        }
        break :parse_filters tmp;
    };

    return raw_filters;
}

/// Use this instead of addRawFilters for a simpler interface
/// Only messages with the corresponding fields will be delivered
/// If this command is called for a second time it overides the first array of filters
pub fn addRawVescFilter(socket: linux.socket_t, filters: []const VescFilter, errno: ?*linux.E) RawFilterError!void {
    // This is to get around the fact that I want to make a mask with the CommandType field while still respecting zig's enum rules
    // I cannot set an enum to 0 if there is no such member and I do not want to add a useless member just for masking
    const _Id = packed struct(u32) {
        vesc_id: u8 = 0,
        command_type: u8 = 0,
        _unused: u13 = 0,
        err: bool = false,
        rtr: bool = false,
        eff: bool = true,
    };

    for (filters) |filter| {
        const raw_filter: CanFilter = parse_filter: {
            var mask: _Id = .{};
            var match: _Id = .{};
            if (filter.vesc_id) |id| {
                mask.vesc_id = 0xff;
                match.vesc_id = id;
            }
            if (filter.command_type) |cmd| {
                mask.command_type = 0xff;
                match.command_type = @intFromEnum(cmd);
            }
            break :parse_filter CanFilter{ .can_id = @bitCast(match), .can_mask = @bitCast(mask) };
        };
        try addRawFilters(socket, &.{raw_filter}, errno);
    }
}

pub fn addBCMFilterSubscription(io: Io, can_socket: linux.socket_t, id: Id) !void {
    const msg_head: can.bcm_msg_head = .{ .opcode = can.RX_SETUP, .can_id = @bitCast(id), .nframes = 0 };

    _ = try io.operate(.{
        .file_write_streaming = .{
            .file = .{ .handle = can_socket, .flags = .{ .nonblocking = false } },
            .data = &.{
                @ptrCast(&msg_head),
            },
        },
    });
}

pub const BCMRXSetupOptions = struct {
    filter_id: Id,
    recveive_all: bool = false, // if true then RX_FILTER_ID will be set and every frame will be delivered, timeout is ignored
    timeout: can.bcm_timeval, // if set, this will be how long it will take for BCM to send a RX_TIMOUT message
    rate_limit: can.bcm_timeval, // if set, this will limit how often a RX_CHANGED will be sent on change
};

pub const BCMConfigError = error{ConfigError};
pub fn BCMConfigureIdReceive(can_socket: linux.socket_t, config: BCMRXSetupOptions, errno: ?*linux.E) BCMConfigError!void {
    std.debug.assert(config.timeout.tv_sec >= 0);
    std.debug.assert(config.timeout.tv_usec >= 0);
    std.debug.assert(config.timeout.tv_usec < 1_000_000);

    std.debug.assert(config.rate_limit.tv_sec >= 0);
    std.debug.assert(config.rate_limit.tv_usec >= 0);
    std.debug.assert(config.rate_limit.tv_usec < 1_000_000);

    var bcm_setup: BCMBuffer(1) = .{
        .msg_head = .{
            .flags = 0,
            .nframes = if (config.recveive_all) 0 else 1,
        },
        .can_frames = .{.{
            .id = config.filter_id,
            .len = 0,
        }},
    };

    bcm_setup.msg_head.opcode = can.RX_SETUP;
    bcm_setup.msg_head.flags |= can.SETTIMER | can.STARTTIMER | can.RX_ANNOUNCE_RESUME;
    bcm_setup.msg_head.can_id = @bitCast(config.filter_id);

    bcm_setup.msg_head.ival1 = config.timeout;
    bcm_setup.msg_head.ival2 = config.rate_limit;

    // the frame is not required if RX_FILTER_ID is true
    const write_size: usize = if (config.recveive_all) @sizeOf(BCMBuffer(0)) else @sizeOf(BCMBuffer(1));
    if (config.recveive_all) {
        bcm_setup.msg_head.flags |= can.RX_FILTER_ID;
    }

    const ret = linux.errno(
        linux.write(
            can_socket,
            std.mem.asBytes(&bcm_setup),
            write_size,
        ),
    );
    if (ret != .SUCCESS) {
        if (errno) |e| {
            e.* = ret;
        }
        return BCMConfigError.ConfigError;
    }
}

test "create can connection" {
    var errno: linux.E = .SUCCESS;
    const socket = try openCANRaw("can0", &errno);
    _ = socket;
    try std.testing.expect(errno == .SUCCESS);
}

test "filtering" {
    const io = std.testing.io;

    const sock1 = try openCANRaw("can0", null);
    defer _ = linux.close(sock1);
    const sock2 = try openCANRaw("can0", null);
    defer _ = linux.close(sock2);

    const filters: [3]VescFilter = .{
        .{ .command_type = .SET_CURRENT },
        .{ .vesc_id = 2 },
        .{ .command_type = .SET_RPM, .vesc_id = 3 },
    };

    const raw_filters = vescFiltersToRawFilters(3, filters);

    try addRawFilters(sock1, &raw_filters, null);

    const accept_frames = [_]CanFrame{
        .{
            .id = .{ .vesc_id = 3, .command_type = .SET_RPM },
            .len = 3,
            .data = .{ .bytes = std.mem.toBytes(@as(u64, 0xdeadbeefdeadbeef)) },
        },
        .{
            .id = .{ .vesc_id = 2, .command_type = .SET_DUTY },
            .len = 0,
        },
        .{
            .id = .{ .vesc_id = 4, .command_type = .SET_CURRENT },
            .len = 1,
        },
    };

    const reject_frames = [_]CanFrame{
        .{
            .id = .{ .vesc_id = 3, .command_type = .SET_DUTY },
            .len = 0,
        },
        .{
            .id = .{ .vesc_id = 4, .command_type = .SET_RPM },
            .len = 0,
        },
    };

    try sendRawCANFrames(io, sock2, &accept_frames);
    try sendRawCANFrames(io, sock2, &reject_frames);

    for (0..accept_frames.len) |_| {
        const read = try readRawCANFrame(io, sock1);
        var found = false;
        for (accept_frames) |frame| {
            if (@as(u128, @bitCast(frame)) == @as(u128, @bitCast(read))) {
                found = true;
                break;
            }
        }
        try std.testing.expect(found);
    }
}

test "timeout test" {
    const io = std.testing.io;

    const can_socket = try openCANBCM("can0", null);
    defer _ = linux.close(can_socket);

    try BCMConfigureIdReceive(
        can_socket,
        .{
            .filter_id = .{ .vesc_id = 17, .command_type = .SET_DUTY },
            .timeout = .{
                .tv_sec = 0,
                .tv_usec = 20 * 1000,
            },
            .rate_limit = .{
                .tv_sec = 0,
                .tv_usec = 0,
            },
            // .recveive_all = true,
        },
        null,
    );

    try io.sleep(.fromMilliseconds(50), .awake);

    const read_frames = try readBCMCANFrame(io, can_socket, 0);
    const expected_frames: BCMBuffer(0) = .{
        .msg_head = .{
            .opcode = can.RX_TIMEOUT,
            .flags = can.STARTTIMER | can.SETTIMER | can.RX_FILTER_ID,
        },
        .can_frames = .{},
    };

    try std.testing.expectEqual(read_frames.msg_head.opcode, expected_frames.msg_head.opcode);
}

test "net receive test" {
    const io = std.testing.io;

    const fake = try simulate_motor.FakeMotor.init("can0");
    defer fake.deinit();

    const can_socket = try openCANBCM("can0", null);
    defer _ = linux.close(can_socket);

    const test_frame = CanFrame.createSetDuty(17, 783);

    var config_errno: linux.E = .SUCCESS;
    try BCMConfigureIdReceive(
        can_socket,
        .{
            .filter_id = .{ .vesc_id = 17, .command_type = .SET_DUTY },
            .timeout = .{
                .tv_sec = 0,
                .tv_usec = 20 * 1000,
            },
            .rate_limit = .{
                .tv_sec = 0,
                .tv_usec = 0,
            },
        },
        &config_errno,
    );
    if (config_errno != .SUCCESS) {
        std.debug.panic(
            "Failed to configure BCM with errno {t}",
            .{config_errno},
        );
    }

    try fake.sendFrame(io, &test_frame);

    try io.sleep(.fromMilliseconds(100), .awake);

    const bcm_buf, const message = try recvmesgBCM2(io, can_socket, 1);
    _ = message;
    try std.testing.expectEqual(test_frame, bcm_buf.can_frames[0]);
}
