const std = @import("std");
const Io = std.Io;
const net = Io.net;
const linux = std.os.linux;
const socket_can = @import("socket_can");
const vesc_datatypes = @import("vesc_datatypes.zig");

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
};

pub const CanFrame = extern struct {
    id: Id,
    len: u8,
    _pad: u8 = 0, // do not use
    _res0: u8 = 0, // do not use
    _len8_dlc: u8 = 0, // VESC does not use this
    data: DataTypes align(8) = .{ .bytes = @splat(0) },
    // data: [8]u8 align(8) = @splat(0),
};

pub const CanFilter = socket_can.can_filter;

pub const VescFilter = struct {
    vesc_id: ?u8 = null,
    command_type: ?CommandType = null,
};

/// if name is null then the returned socket will bind to all CAN networks
/// If an error occures the errno will be set, otherwise it will be left unchanged
pub const SocketCANError = error{ SocketError, IoctlError, BindError };
pub fn openCANRaw(name: ?[]const u8, errno: ?*linux.E) SocketCANError!linux.socket_t {
    const socket_fd: linux.socket_t = get_sock: {
        const rc = linux.socket(linux.PF.CAN, linux.SOCK.RAW, socket_can.CAN_RAW);
        const e: linux.E = linux.errno(rc);
        if (e != .SUCCESS) {
            if (errno) |ptr| {
                ptr.* = e;
            }
            return SocketCANError.SocketError;
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
                return SocketCANError.IoctlError;
            }
            break :get_index ifr.ifru.ivalue;
        } else {
            break :get_index 0;
        }
    };

    const addr: socket_can.sockaddr_can = .{ .can_family = linux.AF.CAN, .can_ifindex = ifindex };

    const bind_rc = linux.errno(linux.bind(socket_fd, @ptrCast(&addr), @sizeOf(@TypeOf(addr))));
    if (bind_rc != .SUCCESS) {
        if (errno) |ptr| {
            ptr.* = bind_rc;
        }
        return SocketCANError.BindError;
    }

    return socket_fd;
}

pub fn sendRAWCANFrame(io: Io, socket: linux.socket_t, can_frame: *const CanFrame) !void {
    _ = try io.operate(.{
        .file_write_streaming = .{
            .file = .{ .handle = socket, .flags = .{ .nonblocking = false } },
            .data = &.{
                @ptrCast(can_frame),
            },
        },
    });
}

pub fn sendRAWCANFrames(io: Io, socket: linux.socket_t, can_frames: []const CanFrame) !void {
    for (can_frames) |frame| {
        try sendRAWCANFrame(io, socket, &frame);
    }
}

pub fn readRAWCANFrame(io: Io, socket: linux.socket_t) !CanFrame {
    var ret: CanFrame = undefined;
    _ = try io.operate(.{ .file_read_streaming = .{
        .file = .{ .handle = socket, .flags = .{ .nonblocking = false } },
        .data = &.{@ptrCast(&ret)},
    } });
    return ret;
}

/// If this command is called for a second time it overides the first array of filters
pub fn addRAWFilters(socket: linux.socket_t, filters: []const CanFilter) void {
    const errno = linux.errno(linux.setsockopt(socket, socket_can.SOL_CAN_RAW, socket_can.CAN_RAW_FILTER, @ptrCast(filters.ptr), @intCast(filters.len * @sizeOf(socket_can.can_filter))));
    if (errno != .SUCCESS) {
        std.debug.print("Failed to setsockopt {}\n", .{errno});
    }
}

pub fn vescFiltersToRAWFilters(comptime N: usize, comptime filters: [N]VescFilter) [N]CanFilter {
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

/// Use this instead of addRAWFilters for a simpler interface
/// Only messages with the corresponding fields will be delivered
/// If this command is called for a second time it overides the first array of filters
pub fn addVescFilter(socket: linux.socket_t, filters: []const VescFilter) void {
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
            std.debug.print("id: {x}\nmk: {x}\n", .{ @as(u32, @bitCast(match)), @as(u32, @bitCast(mask)) });
            break :parse_filter CanFilter{ .can_id = @bitCast(match), .can_mask = @bitCast(mask) };
        };
        std.debug.print("raw: {}\n", .{raw_filter});
        addRAWFilters(socket, &.{raw_filter});
    }
}

test "create can connection" {
    var errno: linux.E = .SUCCESS;
    const socket = try openCANRaw("can0", &errno);
    _ = socket;
    try std.testing.expect(errno == .SUCCESS);
}

test "filtering" {
    var threaded = Io.Threaded.init_single_threaded;
    const io = threaded.io();

    const sock1 = try openCANRaw("can0", null);
    defer _ = linux.close(sock1);
    const sock2 = try openCANRaw("can0", null);
    defer _ = linux.close(sock2);

    const filters: [3]VescFilter = .{
        .{ .command_type = .SET_CURRENT },
        .{ .vesc_id = 2 },
        .{ .command_type = .SET_RPM, .vesc_id = 3 },
    };

    const raw_filters = vescFiltersToRAWFilters(3, filters);

    addRAWFilters(sock1, &raw_filters);

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

    try sendRAWCANFrames(io, sock2, &accept_frames);
    try sendRAWCANFrames(io, sock2, &reject_frames);

    for (0..accept_frames.len) |_| {
        const read = try readRAWCANFrame(io, sock1);
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
