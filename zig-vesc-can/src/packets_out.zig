const std = @import("std");

pub fn vescWrite(T: type, value: f32, scalar: f64) T {
    // vesc does actually cast the passed f32 into a f64
    return std.mem.nativeToBig(T, @intFromFloat(@as(f64, value) * scalar));
}

pub const SetCurrent = packed struct(u64) {
    current: i32, // 1_000
    _unused: u32 = 0,
};

pub const SetCurrentBrake = packed struct(u64) {
    current: i32, // 1_000
    _unused: u32 = 0,
};

pub const SetDuty = packed struct(u64) {
    duty: i32, // Scale of 100_000
    _unused: u32 = 0,
};

pub const SetRPM = packed struct(u64) {
    rpm: i32, // Scale of 1
    _unused: u32 = 0,
};

pub const SetPos = packed struct(u64) {
    degrees: i32, // Scale of 1_000_000
    _unused: u32 = 0,
};
