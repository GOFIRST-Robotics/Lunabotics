const std = @import("std");
const vesc_datatypes = @import("vesc_datatypes.zig");
const CommandType = vesc_datatypes.CommandType;
const packet_io = @import("packet_io.zig");
const FloatInfo = packet_io.FloatInfo;

pub const SetDuty = struct {
    duty: f32,

    pub const type_id = CommandType.SET_DUTY;
    pub const schema = .{
        .duty = FloatInfo{ .underlying = i32, .scalar = 100_000 },
    };
};

pub const SetCurrent = struct {
    current: f32,

    pub const type_id = CommandType.SET_CURRENT;
    pub const schema = .{
        .current = FloatInfo{ .underlying = i32, .scalar = 1_000 },
    };
};

pub const SetCurrentBrake = struct {
    current: f32,

    pub const type_id = CommandType.SET_CURRENT_BRAKE;
    pub const schema = .{
        .current = FloatInfo{ .underlying = i32, .scalar = 1_000 },
    };
};

pub const SetRPM = struct {
    rpm: f32,

    pub const type_id = CommandType.SET_RPM;
    pub const schema = .{
        .rpm = FloatInfo{ .underlying = i32, .scalar = 1 },
    };
};

pub const SetPos = struct {
    degrees: f32,

    pub const type_id = CommandType.SET_POS;
    pub const schema = .{
        .degrees = FloatInfo{ .underlying = i32, .scalar = 1_000_000 },
    };
};

const all_packets_out = [_]packet_io.PacketInfo{
    .{ .name = "set_duty", .packet = SetDuty },
    .{ .name = "set_current", .packet = SetCurrent },
    .{ .name = "set_current_brake", .packet = SetCurrentBrake },
    .{ .name = "set_rpm", .packet = SetRPM },
    .{ .name = "set_pos", .packet = SetPos },
};
const io_types = packet_io.makePacketIO(&all_packets_out);

pub const OutgoingPacketType = io_types.PacketType;
pub const OutgoingPacket = io_types.Packet;
pub const IO = io_types.IO;
