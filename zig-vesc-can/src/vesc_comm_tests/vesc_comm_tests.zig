// Copyright 2026 Michael Foley - foley586@umn.edu
//
// This program is free software: you can redistribute it and/or modify it under
// the terms of the GNU General Public License as published by the Free Software
// Foundation, either version 3 of the License, or (at your option) any later version.
//
// This program is distributed in the hope that it will be useful, but WITHOUT
// ANY WARRANTY; without even the implied warranty of MERCHANTABILITY or FITNESS
// FOR A PARTICULAR PURPOSE. See the GNU General Public License for more details.
//
// You should have received a copy of the GNU General Public License along with
// this program. If not, see <https://www.gnu.org/licenses/>.

const std = @import("std");
const testing = std.testing;
const socket_can = @import("zig-vesc-can").socket_can;
const comm_can = @import("comm_can");

const can_frame = comm_can.can_frame; // C definition
const CanFrame = socket_can.CanFrame; // zig definition

/// True if the frames are equal false if not
fn compareCanFrames(frame1: socket_can.CanFrame, frame2: comm_can.can_frame) !void {
    try testing.expectEqual(frame2.can_id, @as(u32, @bitCast(frame1.id)));
    try testing.expectEqual(frame2.unnamed_0.len, frame1.len);
    for (frame1.data.bytes[0..frame1.len], frame2.data[0..frame1.len]) |b1, b2| {
        try testing.expectEqual(b1, b2);
    }
}

// assert that the custom CAN extern can frame struct is the same as the one defined in linux/can.h
test "CAN Frame is Equal" {
    try testing.expectEqual(@sizeOf(can_frame), @sizeOf(CanFrame));
    try testing.expectEqual(@alignOf(can_frame), @alignOf(CanFrame));

    const zig_fields = @typeInfo(CanFrame).@"struct".fields;
    const c_fields = @typeInfo(can_frame).@"struct".fields;

    try testing.expectEqual(c_fields.len, zig_fields.len);

    inline for (zig_fields, c_fields) |zf, cf| {
        const c_offset = @offsetOf(can_frame, cf.name);
        const zig_offset = @offsetOf(CanFrame, zf.name);
        try testing.expectEqual(c_offset, zig_offset);

        try testing.expectEqual(@sizeOf(cf.type), @sizeOf(zf.type));
    }
}

/// Runs the zig and C encoders over many ids/values and asserts the frames match.
/// `range` keeps value * scale inside i32 so neither side overflows.
fn expectEquivalent(zig_create: fn (u8, f32) CanFrame, c_create: anytype, range: f32) !void {
    const ids = [_]u8{ 0, 1, 12, 127, 255 };
    const fixed = [_]f32{ 0, -0.0, 0.1, -0.1, 1.0 / 3.0, -1.0 / 3.0, 0.5, -0.5, 1, -1, 2.5, -2.5, 55, -55 };
    var prng = std.Random.DefaultPrng.init(0x5EC5);
    const random = prng.random();

    for (ids) |id| {
        for (fixed) |value| try expectFrameEqual(zig_create, c_create, id, value);
        for (0..20_000) |_| {
            const value = (random.float(f32) * 2 - 1) * range;
            try expectFrameEqual(zig_create, c_create, id, value);
        }
    }
}

fn expectFrameEqual(zig_create: fn (u8, f32) CanFrame, c_create: anytype, id: u8, value: f32) !void {
    errdefer std.debug.print("mismatch for vesc_id={d} value={d}\n", .{ id, value });
    c_create(id, value);
    try compareCanFrames(zig_create(id, value), comm_can.created_frame);
}

test "SET_DUTY equivalent" {
    try expectEquivalent(CanFrame.createSetDuty, comm_can.comm_can_set_duty, 20_000);
}

test "SET_CURRENT equivalent" {
    try expectEquivalent(CanFrame.createSetCurrent, comm_can.comm_can_set_current, 2_000_000);
}

test "SET_CURRENT_BRAKE equivalent" {
    try expectEquivalent(CanFrame.createSetCurrentBrake, comm_can.comm_can_set_current_brake, 2_000_000);
}

test "SET_RPM equivalent" {
    try expectEquivalent(CanFrame.createSetRPM, comm_can.comm_can_set_rpm, 2_000_000_000);
}

test "SET_POS equivalent" {
    try expectEquivalent(CanFrame.createSetPos, comm_can.comm_can_set_pos, 2_000);
}
