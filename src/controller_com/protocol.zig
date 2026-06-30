const std = @import("std");
const Io = std.Io;
const net = Io.net;
const linux = std.os.linux;
const mem = std.mem;

// This is purely random and could be any unused port
pub const port = 49153;

pub const TimeStamp = packed struct {
    seconds: isize,
    nano_seconds: isize,

    pub fn fromTimeSpec(time_spec: *const linux.timespec) @This() {
        return .{ .seconds = time_spec.sec, .nano_seconds = time_spec.nsec };
    }
};

/// nano_seconds has to be smaller than 1 billion
pub fn isOlderBy(older: TimeStamp, younger: TimeStamp, seconds: isize, nano_seconds: isize) bool {
    std.debug.assert(nano_seconds < 1_000_000_000);
    const sec_dif = older.seconds - younger.seconds;
    const n_sec_dif = older.nano_seconds - younger.nano_seconds;

    if (sec_dif > seconds) {
        return true;
    } else if (sec_dif == seconds and n_sec_dif > nano_seconds) {
        return true;
    } else {
        return false;
    }
}

pub fn isOlder(older: TimeStamp, younger: TimeStamp) bool {
    if (older.seconds > younger.seconds) {
        return false;
    } else if (older.seconds < younger.seconds) {
        return true;
    } else if (older.nano_seconds < younger.nano_seconds) {
        return true;
    } else {
        return false;
    }
}

/// Parses T from data and converts it to host endianness
/// T must be a type with a known size
pub fn takeFromDgram(T: type, data: []const u8) T {
    std.debug.assert(@sizeOf(T) <= data.len);

    switch (@typeInfo(T)) {
        .@"enum" => |info| {
            return @enumFromInt(mem.bigToNative(info.tag_type, mem.bytesAsValue(info.tag_type, data).*));
        },
        .@"struct" => |info| {
            if (info.backing_integer) |b| {
                return @bitCast(mem.bigToNative(b, mem.bytesAsValue(b, data).*));
            } else {
                @compileError("This function only supports structs that are packed");
            }
        },
        .float => |info| {
            const b = @Int(.unsigned, info.bits);
            return @bitCast(mem.bigToNative(b, mem.bytesAsValue(b, data).*));
        },
        .int => {
            return mem.bigToNative(T, mem.bytesAsValue(T, data));
        },
        else => @compileError("Unsupported Type " ++ @typeName(T)),
    }
}

/// This function will write data in network order to the start of dgram_buf
/// data must be a type with a known size
pub fn writeToDgram(data: anytype, dgram_buf: []u8) void {
    const T = @TypeOf(data);
    std.debug.assert(@sizeOf(T) <= dgram_buf.len);

    switch (@typeInfo(T)) {
        .@"enum" => |info| {
            mem.bytesAsValue(info.tag_type, dgram_buf).* = mem.nativeToBig(info.tag_type, @intFromEnum(data));
        },
        .@"struct" => |info| {
            if (info.backing_integer) |b| {
                mem.bytesAsValue(b, dgram_buf).* = mem.nativeToBig(b, @bitCast(data));
            } else {
                @compileError("This function only supports structs that are packed");
            }
        },
        .float => |info| {
            const b = @Int(.unsigned, info.bits);
            mem.bytesAsValue(b, dgram_buf).* = mem.nativeToBig(b, @bitCast(data));
        },
        .int => {
            mem.bytesAsValue(T, dgram_buf).* = mem.nativeToBig(T, data);
        },
        else => @compileError("Unsupported Type " ++ @typeName(T)),
    }
}

pub const ClientMessageType = enum(u8) {
    controller = 0,
    controller_down,
    stream_deck,
    stream_deck_down,
    ending_connection,
};

// Button order goes in the direction you read
pub const StreamDeckData = packed struct {
    btn_0: bool = false,
    btn_1: bool = false,
    btn_2: bool = false,
    btn_3: bool = false,
    btn_4: bool = false,
    btn_5: bool = false,
    _pad: u2 = 0,

    pub fn format(
        self: @This(),
        writer: *std.Io.Writer,
    ) std.Io.Writer.Error!void {
        const true_char: u8 = 'X';
        const false_char: u8 = ' ';
        try writer.print(
            \\ -------------{0s}
            \\ | {1c} | {2c} | {3c} |{0s}
            \\ -------------{0s}
            \\ | {4c} | {5c} | {6c} |{0s}
            \\ -------------{0s}
            \\
        , .{
            "\x1B[K",
            if (self.btn_0) true_char else false_char,
            if (self.btn_1) true_char else false_char,
            if (self.btn_2) true_char else false_char,
            if (self.btn_3) true_char else false_char,
            if (self.btn_4) true_char else false_char,
            if (self.btn_5) true_char else false_char,
        });
    }
};

pub const ControllerData = packed struct {
    buttons: packed struct {
        north: bool = false,
        east: bool = false,
        south: bool = false,
        west: bool = false,
    } = .{},
    d_pad: packed struct {
        up: bool = false,
        left: bool = false,
        right: bool = false,
        down: bool = false,
    } = .{},

    l1: bool = false, // left bumper
    l2: bool = false, // left trigger
    l3: bool = false, // left stick pressed in

    r1: bool = false, // right bumper
    r2: bool = false, // right trigger
    r3: bool = false, // right stick pressed in

    left_stick: packed struct {
        x: f32 = 0,
        y: f32 = 0,
    } = .{},

    right_stick: packed struct {
        x: f32 = 0,
        y: f32 = 0,
    } = .{},
    _pad: u2 = 0,

    pub fn format(
        self: @This(),
        writer: *std.Io.Writer,
    ) std.Io.Writer.Error!void {
        try writer.print(
            \\buttons:{0s}
            \\  north: {1}{0s}
            \\  east:  {2}{0s}
            \\  south: {3}{0s}
            \\  west:  {4}{0s}
            \\dpad:{0s}
            \\  up:    {5}{0s}
            \\  right: {6}{0s}
            \\  down:  {7}{0s}
            \\  left:  {8}{0s}
            \\L1: {9}{0s}
            \\L2: {10}{0s}
            \\L3: {11}{0s}
            \\R1: {12}{0s}
            \\R2: {13}{0s}
            \\R3: {14}{0s}
            \\Left Stick:{0s}
            \\  x: {15d}{0s}
            \\  y: {16d}{0s}
            \\Right Stick:{0s}
            \\  x: {17d}{0s}
            \\  y: {18d}{0s}
        ,
            .{
                "\x1B[K",
                self.buttons.north,
                self.buttons.east,
                self.buttons.south,
                self.buttons.west,
                self.d_pad.up,
                self.d_pad.right,
                self.d_pad.down,
                self.d_pad.left,
                self.l1,
                self.l2,
                self.l3,
                self.r1,
                self.r2,
                self.r3,
                self.left_stick.x,
                self.left_stick.y,
                self.right_stick.x,
                self.right_stick.y,
            },
        );
    }
};
