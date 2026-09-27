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
const MFR = @import("MFR");
const vesc_datatypes = MFR.vesc_datatypes;
const socket_can = MFR.socket_can;
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

test "equal" {
    const id = 12;
    const current = 55;
    // const can_frame1 = vesc_datatypes.SetCurrent.create(id, current);
    const can_frame1: CanFrame = .{
        .id = .{
            .vesc_id = id,
            .command_type = .SET_CURRENT,
        },
        .len = 4,
        .data = .{ .set_current = .create(current) },
    };
    comm_can.comm_can_set_current(id, current);
    const can_frame2 = comm_can.created_frame;

    try compareCanFrames(can_frame1, can_frame2);
}
