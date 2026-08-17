const std = @import("std");
const Io = std.Io;
const linux = std.os.linux;
const zig_vesc_can = @import("zig-vesc-can");
const socket_can = zig_vesc_can.socket_can;
const vesc_datatypes = zig_vesc_can.vesc_datatypes;
const main = @import("main.zig");
const MotorInfo = main.MotorInfo;
const MotorMap = main.MotorMap;
const bigToNative = std.mem.bigToNative;
const comptimePrint = std.fmt.comptimePrint;

const Dashboard = @This();

const esc = "\x1B";
const clear_modes = "\x1B[0m";
const red = "[31m";
const green = "[32m";
const tile_height = 10;

layout: []const []const u8,
stdout: Io.File,
write_buf: [1024 * 3]u8 = @splat(0),
writer: Io.File.Writer,
io: Io,

pub fn init(self: *@This(), io: Io, layout: []const []const u8) void {
    self.* = .{
        .stdout = Io.File.stdout(),
        .io = io,
        .layout = layout,
        .writer = undefined,
        .write_buf = @splat(0),
    };

    self.writer = .init(self.stdout, self.io, &self.write_buf);
}

pub fn draw(self: *@This(), map: *const MotorMap) !void {
    const interface = &self.writer.interface;
    const winsize = getTermSize();
    try clearAndReset(self);

    for (self.layout) |row| {
        const tile_width: u16 = @intCast(winsize.col / row.len);

        for (row, 0..) |vesc_id, i| {
            try self.savePos();
            const info = map.get(vesc_id).?;

            // Name
            var name_buf: [50]u8 = @splat(0);
            var name_buf_writer: Io.Writer = .fixed(&name_buf);
            try name_buf_writer.writeAll(info.name);
            try name_buf_writer.print(" ({d})", .{vesc_id});
            try interface.printAscii(
                name_buf_writer.buffered(),
                .{
                    .width = tile_width,
                    .fill = ' ',
                    .alignment = .center,
                },
            );

            // Command Name
            try self.moveLeft(tile_width);
            try self.moveDown(1);
            try interface.printAscii(
                @tagName(info.last_command.command),
                .{
                    .alignment = .center,
                    .fill = ' ',
                    .width = tile_width,
                },
            );

            // Commmand Value
            try self.moveLeft(tile_width);
            try self.moveDown(1);
            var format_buf: [100]u8 = @splat(0);
            // use this writer to format the command info
            // padding is handled after the switch statement
            var format_buf_writer: Io.Writer = .fixed(&format_buf);
            switch (info.last_command.command) {
                .SET_DUTY => |v| {
                    const duty = @as(f32, @floatFromInt(bigToNative(u32, v.duty))) / 100_000;
                    try format_buf_writer.printFloat(
                        duty,
                        .{
                            .precision = 0,
                        },
                    );
                    try format_buf_writer.writeByte('%');
                },
                .SET_CURRENT => |v| {
                    const current = @as(f32, @floatFromInt(bigToNative(i32, v.current))) / 1_000;
                    try format_buf_writer.printFloat(
                        current,
                        .{
                            .precision = 0,
                        },
                    );
                    try format_buf_writer.writeByte('A');
                },
                .SET_RPM => |v| {
                    const rpm: f32 = @floatFromInt(bigToNative(i32, v.rpm));
                    try format_buf_writer.printFloat(
                        rpm,
                        .{
                            .precision = 0,
                        },
                    );
                    try format_buf_writer.writeAll(" RPM");
                },
                .SET_POS => |v| {
                    const pos = @as(f32, @floatFromInt(bigToNative(i32, v.degrees))) / 1_000_000;
                    try format_buf_writer.printFloat(
                        pos,
                        .{
                            .precision = 0,
                        },
                    );
                    try format_buf_writer.writeAll(" Degrees");
                },
            }

            try format_buf_writer.writeByte(' ');
            if (info.last_command.timestamp.sec > 0 or info.last_command.timestamp.nsec > 20_000) {
                try format_buf_writer.writeAll(esc ++ red);
            } else {
                try format_buf_writer.writeAll(esc ++ green);
            }
            try format_buf_writer.printInt(
                getElapsedTimems(info.last_command.timestamp),
                10,
                .lower,
                .{ .precision = 0 },
            );
            try format_buf_writer.writeAll(clear_modes);
            try format_buf_writer.writeAll("ms");
            const left_padding: u16 = @intCast((tile_width - format_buf_writer.buffered().len) / 2);
            const right_padding: u16 = @intCast((tile_width - format_buf_writer.buffered().len + 1) / 2);
            try interface.splatByteAll(' ', left_padding);
            try interface.writeAll(format_buf_writer.buffered());
            try interface.splatByteAll(' ', right_padding);

            // Status
            // Currently this is only for Status packet 1 and 5
            var status_buf: [100]u8 = @splat(0);
            var status_buf_writer: Io.Writer = .fixed(&status_buf);
            try self.moveLeft(tile_width);
            try self.moveDown(1);

            const is_alive: bool = blk: {
                inline for (std.meta.fields(@TypeOf(info.status))) |f| {
                    const status = @field(info.status, f.name);
                    if (status) |s| {
                        if (!s.is_alive) {
                            break :blk false;
                        }
                    }
                }
                break :blk true;
            };

            if (is_alive) {
                try status_buf_writer.writeAll(esc ++ green ++ "Alive" ++ clear_modes);
            } else {
                try status_buf_writer.writeAll(esc ++ red ++ "DEAD" ++ clear_modes);
                try status_buf_writer.writeAll("(Status");

                if (info.status.status1) |status1| {
                    if (!status1.is_alive) {
                        try status_buf_writer.print("{d},", .{1});
                    }
                }
                if (info.status.status5) |status5| {
                    if (!status5.is_alive) {
                        try status_buf_writer.print("{d}", .{5});
                    }
                }

                try status_buf_writer.writeAll(")");
            }

            try interface.printAscii(status_buf_writer.buffered(), .{
                .fill = ' ',
                .alignment = .center,
                .width = tile_width,
            });

            if (info.status.status1) |status1| {
                if (status1.is_alive) {
                    const stat = status1.value;

                    try self.moveLeft(tile_width);
                    try self.moveDown(1);
                    const erpm_format = "ERPM: ";
                    try interface.print(erpm_format, .{});
                    try interface.printFloat(stat.erpm, .{
                        .precision = 1,
                        .fill = ' ',
                        .alignment = .left,
                        .width = tile_width - erpm_format.len,
                    });

                    try self.moveLeft(tile_width);
                    try self.moveDown(1);
                    const duty_cycle_format = "Duty Cycle: ";
                    try interface.print(duty_cycle_format, .{});
                    try interface.printFloat(stat.duty_cycle, .{
                        .precision = 1,
                        .fill = ' ',
                        .alignment = .left,
                        .width = tile_width - duty_cycle_format.len,
                    });

                    try self.moveLeft(tile_width);
                    try self.moveDown(1);
                    const current_format = "Current: ";
                    try interface.print(current_format, .{});
                    try interface.printFloat(stat.current, .{
                        .precision = 1,
                        .fill = ' ',
                        .alignment = .left,
                        .width = tile_width - current_format.len,
                    });
                } else {
                    try self.moveLeft(tile_width);
                    try self.moveDown(1);
                    const erpm_format = "ERPM: ---";
                    try interface.printAscii(
                        erpm_format,
                        .{
                            .width = tile_width,
                            .alignment = .left,
                            .fill = ' ',
                        },
                    );

                    try self.moveLeft(tile_width);
                    try self.moveDown(1);
                    const duty_cycle_format = "Duty Cycle: ---";
                    try interface.printAscii(
                        duty_cycle_format,
                        .{
                            .width = tile_width,
                            .alignment = .left,
                            .fill = ' ',
                        },
                    );

                    try self.moveLeft(tile_width);
                    try self.moveDown(1);
                    const current_format = "Current: ---";
                    try interface.printAscii(
                        current_format,
                        .{
                            .width = tile_width,
                            .alignment = .left,
                            .fill = ' ',
                        },
                    );
                }
            }

            if (info.status.status5) |status5| {
                if (status5.is_alive) {
                    const stat = status5.value;

                    try self.moveLeft(tile_width);
                    try self.moveDown(1);
                    const tachometer_format = "Tachometer: ";
                    try interface.print(tachometer_format, .{});
                    try interface.printFloat(stat.tachometer, .{
                        .alignment = .left,
                        .width = tile_width - tachometer_format.len,
                        .precision = 1,
                    });

                    try self.moveLeft(tile_width);
                    try self.moveDown(1);
                    const volts_in_format = "Volts in: ";
                    try interface.print(volts_in_format, .{});
                    try interface.printFloat(stat.volts_in, .{
                        .alignment = .left,
                        .width = tile_width - volts_in_format.len,
                        .precision = 1,
                    });
                } else {
                    try self.moveLeft(tile_width);
                    try self.moveDown(1);
                    const tachometer_format = "Tachometer: ---";
                    try interface.printAscii(tachometer_format, .{
                        .alignment = .left,
                        .width = tile_width,
                    });

                    try self.moveLeft(tile_width);
                    try self.moveDown(1);
                    const volts_in_format = "Volts in: ---";
                    try interface.printAscii(volts_in_format, .{
                        .alignment = .left,
                        .width = tile_width,
                    });
                }
            }

            try self.restorePos();
            if (i != row.len - 1) {
                try self.moveRight(tile_width);
            }
        }

        try self.moveDown(tile_height);
        try self.moveToColumn(0);
    }

    try self.writer.interface.flush();
}

fn getTermSize() std.posix.winsize {
    var winsize: std.posix.winsize = undefined;
    if (linux.errno(
        linux.ioctl(
            Io.File.stdout().handle,
            linux.T.IOCGWINSZ,
            @intFromPtr(&winsize),
        ),
    ) != .SUCCESS) {
        std.debug.panic("Failed to get window size", .{});
    }
    return winsize;
}

fn clearAndReset(self: *@This()) !void {
    try self.writer.interface.writeAll(esc ++ "[H" ++ esc ++ "[2J");
}

fn savePos(self: *@This()) !void {
    try self.writer.interface.writeAll(esc ++ " 7");
    try self.writer.interface.writeAll(esc ++ "[s");
}

fn restorePos(self: *@This()) !void {
    try self.writer.interface.writeAll(esc ++ " 8");
    try self.writer.interface.writeAll(esc ++ "[u");
}

fn moveLeft(self: *@This(), n: u16) !void {
    try self.writer.interface.print(esc ++ "[{d}D", .{n});
}

fn moveRight(self: *@This(), n: u16) !void {
    try self.writer.interface.print(esc ++ "[{d}C", .{n});
}

fn moveDown(self: *@This(), n: u16) !void {
    try self.writer.interface.print(esc ++ "[{d}B", .{n});
}

fn moveToColumn(self: *@This(), n: u16) !void {
    try self.writer.interface.print(esc ++ "[{d}G", .{n});
}

fn getElapsedTimems(since: linux.kernel_timespec) i64 {
    var current_time: linux.timespec = undefined;
    const errno = linux.errno(linux.clock_gettime(.REALTIME, &current_time));
    if (errno != .SUCCESS) {
        std.debug.panic("Failed to get time with clock gettime, Errno: {t}", .{errno});
    }

    var sec_dif = current_time.sec - since.sec;
    var nsec_dif = current_time.nsec - since.nsec;

    if (nsec_dif < 0) {
        sec_dif -= 1;
        nsec_dif += 1_000_000_000;
    }

    return sec_dif * 1_000 + @divFloor(nsec_dif, 1_000_000);
}
