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
const comm_can = @import("comm_can");
const zig_vesc_can = @import("zig-vesc-can");
const vesc_can = zig_vesc_can.vesc_can;
const vesc_packets = zig_vesc_can.vesc_packets;

const cCanFrame = comm_can.can_frame; // C definition
const zigCanFrame = vesc_can.CanFrame; // zig definition

/// True if the frames are equal false if not
fn compareCanFrames(zig_frame: zigCanFrame, c_frame: cCanFrame) !void {
    try testing.expectEqual(c_frame.can_id, @as(u32, @bitCast(zig_frame.id)));
    try testing.expectEqual(c_frame.unnamed_0.len, zig_frame.len);
    try testing.expectEqualSlices(
        u8,
        c_frame.data[0..c_frame.unnamed_0.len],
        zig_frame.data[0..zig_frame.len],
    );
}

// assert that the custom CAN extern can frame struct is the same as the one defined in linux/can.h
test "CAN Frame is Equal" {
    try testing.expectEqual(@sizeOf(cCanFrame), @sizeOf(zigCanFrame));
    try testing.expectEqual(@alignOf(cCanFrame), @alignOf(zigCanFrame));

    const zig_fields = @typeInfo(zigCanFrame).@"struct".fields;
    const c_fields = @typeInfo(cCanFrame).@"struct".fields;

    try testing.expectEqual(c_fields.len, zig_fields.len);

    inline for (zig_fields, c_fields) |zf, cf| {
        const c_offset = @offsetOf(cCanFrame, cf.name);
        const zig_offset = @offsetOf(zigCanFrame, zf.name);
        try testing.expectEqual(c_offset, zig_offset);

        try testing.expectEqual(@sizeOf(cf.type), @sizeOf(zf.type));
    }
}

/// Runs the zig and C encoders over many ids/values and asserts the frames match.
/// `range` keeps value * scale inside i32 so neither side overflows.
fn expectEquivalent(zig_create: type, c_create: anytype, range: f32) !void {
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

fn expectFrameEqual(zig_create: type, c_create: anytype, id: u8, value: f32) !void {
    errdefer std.debug.print("mismatch for vesc_id={d} value={d}\n", .{ id, value });
    c_create(id, value);
    try compareCanFrames(zig_create.create(id, value), comm_can.created_frame);
}

test "SET_DUTY equivalent" {
    try expectEquivalent(struct {
        fn create(id: u8, value: f32) zigCanFrame {
            return vesc_packets.serializeIntoFrame(id, .{ .set_duty = .{ .duty = value } });
        }
    }, comm_can.comm_can_set_duty, 20_000);
}

test "SET_CURRENT equivalent" {
    try expectEquivalent(struct {
        fn create(id: u8, value: f32) zigCanFrame {
            return vesc_packets.serializeIntoFrame(id, .{ .set_current = .{ .current = value } });
        }
    }, comm_can.comm_can_set_current, 2_000_000);
}

test "SET_CURRENT_BRAKE equivalent" {
    try expectEquivalent(struct {
        fn create(id: u8, value: f32) zigCanFrame {
            return vesc_packets.serializeIntoFrame(id, .{ .set_current_brake = .{ .current = value } });
        }
    }, comm_can.comm_can_set_current_brake, 2_000_000);
}

test "SET_RPM equivalent" {
    try expectEquivalent(struct {
        fn create(id: u8, value: f32) zigCanFrame {
            return vesc_packets.serializeIntoFrame(id, .{ .set_rpm = .{ .rpm = value } });
        }
    }, comm_can.comm_can_set_rpm, 2_000_000_000);
}

test "SET_POS equivalent" {
    try expectEquivalent(struct {
        fn create(id: u8, value: f32) zigCanFrame {
            return vesc_packets.serializeIntoFrame(id, .{ .set_pos = .{ .degrees = value } });
        }
    }, comm_can.comm_can_set_pos, 2_000);
}

test "duty cycle" {
    // test duty cycle for values [-1.0, 1.0]
    const step_size: f32 = 0.01;
    const num_steps: isize = @round(1.0 / step_size);

    for (0..num_steps) |step| {
        const duty: f32 = std.math.lerp(@as(f32, -1.0), @as(f32, 1.0), @as(f32, @floatFromInt(step)) * step_size);
        for (0..std.math.maxInt(u8)) |id| {
            comm_can.comm_can_set_duty(@intCast(id), duty);
            const c_frame: cCanFrame = comm_can.created_frame;
            const zig_frame: zigCanFrame = vesc_packets.serializeIntoFrame(@intCast(id), .{ .set_duty = .{ .duty = duty } });
            try compareCanFrames(zig_frame, c_frame);
        }
    }
}

test "current" {
    // test current for values [-10000.0, 10000.0]
    const step_size: f32 = 0.01;
    const num_steps: isize = @round(10000.0 / step_size);

    for (0..num_steps) |step| {
        const current: f32 = std.math.lerp(@as(f32, -10000.0), @as(f32, 10000.0), @as(f32, @floatFromInt(step)) / @as(f32, num_steps));
        for (0..std.math.maxInt(u8)) |id| {
            comm_can.comm_can_set_current(@intCast(id), current);
            const c_frame: cCanFrame = comm_can.created_frame;
            const zig_frame: zigCanFrame = vesc_packets.serializeIntoFrame(@intCast(id), .{ .set_current = .{ .current = current } });
            try compareCanFrames(zig_frame, c_frame);
        }
    }
}
