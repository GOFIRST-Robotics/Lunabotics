const std = @import("std");
const Io = std.Io;
const log = std.log;
const protocol = @import("protocol.zig");
const input = @import("input_headers");
const linux = std.os.linux;

pub const logitech_name = "Logitech_Logitech_Dual_Action";
pub const stream_deck_name = "Stream_Deck";

pub const FindControllerError = error{FileOperationFailure};
/// This function searches in /dev/input/by-id for a event joystick in with dev_name
/// The dev_name only needs to be a substring found in the path
/// null is returned if the controller cannot be found
/// When Canceled null is returned
pub fn findDevice(io: Io, dev_name: []const u8) FindControllerError!?Io.File {
    var name: [1024:0]u8 = @splat(0);
    std.debug.assert(dev_name.len < name.len);

    const con_file = find_controller: {
        const inputs_path: []const u8 = "/dev/input/by-id/";
        var inputs_dir = Io.Dir.openDirAbsolute(io, inputs_path, .{ .iterate = true }) catch |err| switch (err) {
            error.Canceled => return null,
            else => {
                log.err("Failed to open '{s}' directory; Error {}", .{ inputs_path, err });
                return error.FileOperationFailure;
            },
        };
        defer inputs_dir.close(io);
        var iter = inputs_dir.iterate();
        while (iter.next(io) catch |err| switch (err) {
            error.Canceled => return null,
            else => {
                log.err("Failed to iterate through '{s}'; Error {}", .{ inputs_path, err });
                return error.FileOperationFailure;
            },
        }) |file| {
            // if (std.mem.find(u8, file.name, "event-joystick")) |_| {
            if (std.mem.find(u8, file.name, "event")) |_| {
                if (std.mem.find(u8, file.name, dev_name)) |_| {
                    @memcpy(name[0..inputs_path.len], inputs_path);
                    @memcpy(name[inputs_path.len .. inputs_path.len + file.name.len], file.name);

                    break :find_controller inputs_dir.openFile(io, file.name, .{}) catch |err| switch (err) {
                        error.Canceled => return null,
                        else => return error.FileOperationFailure,
                    };
                }
            }
        }
        return null;
    };

    return con_file;
}

fn normalizeAndClamp(val: i32, min: i32, max: i32) f32 {
    const norm = 2 * (@as(f32, @floatFromInt(val - min)) / @as(f32, @floatFromInt(max - min))) - 1;
    // paranoid operation
    return std.math.clamp(norm, -1.0, 1.0);
}

const logitechAxisMin = 0;
const logitechAxisMax = 255;
pub const ControllerReadError = error{ LostController, ReadError };
pub fn readFromLogitechController(io: Io, file: Io.File, previous_data: protocol.ControllerData) ControllerReadError!struct { time: struct { sec: isize, nsec: isize }, data: protocol.ControllerData } {
    var event: input.input_event = undefined;
    var time: linux.timespec = undefined;
    const get_time_rc = linux.errno(linux.clock_gettime(linux.clockid_t.REALTIME, &time));
    if (get_time_rc != .SUCCESS) {
        std.debug.panic("Failed to get time with function 'linux.clock_gettime', errno: {d}\n", .{get_time_rc});
    }

    var data = previous_data;

    while (true) {
        const len_read = file.readStreaming(io, &.{@ptrCast(&event)}) catch |err| switch (err) {
            error.Unexpected => {
                const errno: c_int = std.c._errno().*;
                // This has to be done because the errno NODEV is not yet handled by zig when reading from a file
                // NODEV is 19
                if (errno == @intFromEnum(linux.E.NODEV)) {
                    return ControllerReadError.LostController;
                } else {
                    log.warn("An unexpected error occured while reading from logitech controller. Errno: {d}\n", .{errno});
                    return ControllerReadError.ReadError;
                }
            },
            error.Canceled => {
                return .{ .time = .{ .sec = time.sec, .nsec = time.nsec }, .data = data };
            },
            else => {
                log.warn("A read error occured while reading from logitech controller. Error: {s}\n", .{@errorName(err)});
                return ControllerReadError.ReadError;
            },
        };

        std.debug.assert(len_read == @sizeOf(input.input_event));

        switch (event.type) {
            input.EV_KEY => {
                switch (event.code) {
                    input.BTN_TOP => data.buttons.north = event.value == 1,
                    input.BTN_THUMB2 => data.buttons.east = event.value == 1,
                    input.BTN_THUMB => data.buttons.south = event.value == 1,
                    input.BTN_TRIGGER => data.buttons.west = event.value == 1,
                    input.BTN_TOP2 => data.l1 = event.value == 1,
                    input.BTN_PINKIE => data.r1 = event.value == 1,
                    input.BTN_BASE => data.l2 = event.value == 1,
                    input.BTN_BASE2 => data.r2 = event.value == 1,
                    input.BTN_BASE5 => data.l3 = event.value == 1,
                    input.BTN_BASE6 => data.r3 = event.value == 1,
                    else => {},
                }
            },
            input.EV_ABS => {
                switch (event.code) {
                    input.ABS_HAT0X => {
                        if (event.value == -1) {
                            data.d_pad.left = true;
                            data.d_pad.right = false;
                        } else if (event.value == 1) {
                            data.d_pad.left = false;
                            data.d_pad.right = true;
                        } else {
                            data.d_pad.left = false;
                            data.d_pad.right = false;
                        }
                    },
                    input.ABS_HAT0Y => {
                        if (event.value == -1) {
                            data.d_pad.up = true;
                            data.d_pad.down = false;
                        } else if (event.value == 1) {
                            data.d_pad.up = false;
                            data.d_pad.down = true;
                        } else {
                            data.d_pad.up = false;
                            data.d_pad.down = false;
                        }
                    },
                    input.ABS_X => {
                        data.left_stick.x = normalizeAndClamp(event.value, logitechAxisMin, logitechAxisMax);
                    },
                    input.ABS_Y => {
                        data.left_stick.y = normalizeAndClamp(event.value, logitechAxisMin, logitechAxisMax);
                    },
                    input.ABS_Z => {
                        data.right_stick.x = normalizeAndClamp(event.value, logitechAxisMin, logitechAxisMax);
                    },
                    input.ABS_RZ => {
                        data.right_stick.y = normalizeAndClamp(event.value, logitechAxisMin, logitechAxisMax);
                    },
                    else => {},
                }
            },
            else => {},
        }

        time = .{ .sec = event.time.tv_sec, .nsec = microSecondToNanoSecond(event.time.tv_usec) };
    }
}

pub fn readFromStreamDeck(io: Io, file: Io.File, previous_data: protocol.StreamDeckData) ControllerReadError!struct { time: struct { sec: isize, nsec: isize }, data: protocol.StreamDeckData } {
    var event: input.input_event = undefined;
    var time: linux.timespec = undefined;
    const get_time_rc = linux.errno(linux.clock_gettime(linux.clockid_t.REALTIME, &time));
    if (get_time_rc != .SUCCESS) {
        std.debug.panic("Failed to get time with function 'linux.clock_gettime', errno: {d}\n", .{get_time_rc});
    }

    var data = previous_data;

    while (true) {
        const len_read = file.readStreaming(io, &.{@ptrCast(&event)}) catch |err| switch (err) {
            error.Unexpected => {
                const errno: c_int = std.c._errno().*;
                // This has to be done because the errno NODEV is not yet handled by zig when reading from a file
                // NODEV is 19
                if (errno == @intFromEnum(linux.E.NODEV)) {
                    return ControllerReadError.LostController;
                } else {
                    log.warn("An unexpected error occured while reading from logitech controller. Errno: {d}\n", .{errno});
                    return ControllerReadError.ReadError;
                }
            },
            error.Canceled => {
                return .{ .time = .{ .sec = time.sec, .nsec = time.nsec }, .data = data };
            },
            else => {
                log.warn("A read error occured while reading from logitech controller; {}\n", .{err});
                return ControllerReadError.ReadError;
            },
        };

        std.debug.assert(len_read == @sizeOf(input.input_event));

        switch (event.type) {
            input.EV_KEY => {
                switch (event.code) {
                    input.KEY_UNKNOWN => data.btn_0 = event.value == 1,
                    input.BTN_0 => data.btn_1 = event.value == 1,
                    input.BTN_1 => data.btn_2 = event.value == 1,
                    input.BTN_2 => data.btn_3 = event.value == 1,
                    input.BTN_3 => data.btn_4 = event.value == 1,
                    input.BTN_4 => data.btn_5 = event.value == 1,
                    else => {},
                }
            },
            else => {},
        }

        time = .{ .sec = event.time.tv_sec, .nsec = microSecondToNanoSecond(event.time.tv_usec) };
    }
}

fn microSecondToNanoSecond(usec: i64) isize {
    return usec * 1_000;
}
