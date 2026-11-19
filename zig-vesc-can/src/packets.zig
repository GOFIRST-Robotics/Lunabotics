const std = @import("std");
const can = @import("can.zig");
const codecs = @import("codecs.zig");

const CanFrame = can.CanFrame;
const VescCanId = can.VescCanId;
const BufReader = codecs.BufReader;
const BufWriter = codecs.BufWriter;

pub const PacketType = enum(u8) {
    status_1 = 9,
    status_2 = 14,
    status_3 = 15,
    status_4 = 16,
    status_5 = 27,
    status_6 = 58,
    notify_boot = 57,
    set_duty = 0,
    set_current = 1,
    set_current_brake = 2,
    set_rpm = 3,
    set_pos = 4,
    ping = 17,
    pong = 18,
    fill_rx_buffer = 5,
    fill_rx_buffer_long = 6,
    process_rx_buffer = 7,
    process_short_buffer = 8,
};

pub const Packet = union(PacketType) {
    // Status signals (VESC -> controller)
    status_1: Status1,
    status_2: Status2,
    status_3: Status3,
    status_4: Status4,
    status_5: Status5,
    status_6: Status6,
    notify_boot: NotifyBoot,

    // Control requests (controller -> VESC)
    set_duty: SetDuty,
    set_current: SetCurrent,
    set_current_brake: SetCurrentBrake,
    set_rpm: SetRPM,
    set_pos: SetPos,

    // Pings
    ping: Ping,
    pong: Pong,

    // Command fragmentation (bidirectional)
    fill_rx_buffer: FillRxBuffer,
    fill_rx_buffer_long: FillRxBufferLong,
    process_rx_buffer: ProcessRxBuffer,
    process_short_buffer: ProcessShortBuffer,
};

pub const Status1 = struct {
    erpm: f32,
    current: f32,
    duty_cycle: f32,

    pub const Codec = codecs.Struct(@This(), .{
        .erpm = codecs.FixedFloat(i32, @as(f64, 1)),
        .current = codecs.FixedFloat(i16, @as(f64, 10)),
        .duty_cycle = codecs.FixedFloat(i16, @as(f64, 1_000)),
    });
};

pub const Status2 = struct {
    amp_hours: f32,
    amp_hours_chg: f32,

    pub const Codec = codecs.Struct(@This(), .{
        .amp_hours = codecs.FixedFloat(i32, @as(f64, 10_000)),
        .amp_hours_chg = codecs.FixedFloat(i32, @as(f64, 10_000)),
    });
};

pub const Status3 = struct {
    watt_hours: f32,
    watt_hours_chg: f32,

    pub const Codec = codecs.Struct(@This(), .{
        .watt_hours = codecs.FixedFloat(i32, @as(f64, 10_000)),
        .watt_hours_chg = codecs.FixedFloat(i32, @as(f64, 10_000)),
    });
};

pub const Status4 = struct {
    temp_FET: f32,
    temp_motor: f32,
    current_in: f32,
    PID_pos: f32,

    pub const Codec = codecs.Struct(@This(), .{
        .temp_FET = codecs.FixedFloat(i16, @as(f64, 10)),
        .temp_motor = codecs.FixedFloat(i16, @as(f64, 10)),
        .current_in = codecs.FixedFloat(i16, @as(f64, 10)),
        .PID_pos = codecs.FixedFloat(i16, @as(f64, 50)),
    });
};

pub const Status5 = struct {
    tachometer: f32,
    volts_in: f32,

    pub const Codec = codecs.Struct(@This(), .{
        .tachometer = codecs.FixedFloat(i32, @as(f64, 6)),
        .volts_in = codecs.FixedFloat(i16, @as(f64, 10)),
    });
};

pub const Status6 = struct {
    ADC1: f32,
    ADC2: f32,
    ADC3: f32,
    PPM: f32,

    pub const Codec = codecs.Struct(@This(), .{
        .ADC1 = codecs.FixedFloat(i16, @as(f64, 1_000)),
        .ADC2 = codecs.FixedFloat(i16, @as(f64, 1_000)),
        .ADC3 = codecs.FixedFloat(i16, @as(f64, 1_000)),
        .PPM = codecs.FixedFloat(i16, @as(f64, 1_000)),
    });
};

pub const NotifyBoot = struct {
    hw_name: []const u8,

    pub const Codec = codecs.Struct(@This(), .{
        .hw_name = codecs.RemainingBytes,
    });
};

pub const SetDuty = struct {
    duty: f32,

    pub const Codec = codecs.Struct(@This(), .{
        .duty = codecs.FixedFloat(i32, @as(f64, 100_000)),
    });
};

pub const SetCurrent = struct {
    current: f32,
    current_off_delay: ?f32 = null,

    pub const Codec = codecs.Struct(@This(), .{
        .current = codecs.FixedFloat(i32, @as(f64, 1_000)),
        .current_off_delay = codecs.Optional(codecs.FixedFloat(i16, @as(f64, 1_000))),
    });
};

pub const SetCurrentBrake = struct {
    current: f32,

    pub const Codec = codecs.Struct(@This(), .{
        .current = codecs.FixedFloat(i32, @as(f64, 1_000)),
    });
};

pub const SetRPM = struct {
    rpm: f32,

    pub const Codec = codecs.Struct(@This(), .{
        .rpm = codecs.FixedFloat(i32, @as(f64, 1)),
    });
};

pub const SetPos = struct {
    degrees: f32,

    pub const Codec = codecs.Struct(@This(), .{
        .degrees = codecs.FixedFloat(i32, @as(f64, 1_000_000)),
    });
};

pub const Ping = struct {
    sender_id: u8,

    pub const Codec = codecs.Struct(@This(), .{
        .sender_id = codecs.Int(u8),
    });
};

pub const HardwareType = enum(u8) {
    vesc = 0,
    vesc_bms = 1,
    custom_module = 2,

    pub const Codec = codecs.IntEnum(HardwareType);
};

pub const Pong = struct {
    sender_id: u8,
    hardware_type: ?HardwareType,

    pub const Codec = codecs.Struct(@This(), .{
        .sender_id = codecs.Int(u8),
        .hardware_type = codecs.Optional(HardwareType.Codec),
    });
};

pub const BufferSendMode = enum(u8) {
    request = 0, // Default; recipient VESC processes the command, replies use mode `response`
    response = 1, // Response from a request; if sent to a VESC, it is forwarded to its last used interface (for VESC tool)
    request_dont_reply = 2, // Same as request, but no replies are sent
    request_reply_as_request = 3, // Same as request, but replies are sent with mode `request`. This is used for VESC-to-VESC communication

    pub const Codec = codecs.IntEnum(@This());
};

pub const ProcessShortBuffer = struct {
    sender_id: u8,
    send_mode: BufferSendMode,
    data: []const u8,

    pub const Codec = codecs.Struct(@This(), .{
        .sender_id = codecs.Int(u8),
        .send_mode = BufferSendMode.Codec,
        .data = codecs.RemainingBytes,
    });
};

pub const FillRxBuffer = struct {
    base_index: u8,
    data: []const u8,

    pub const Codec = codecs.Struct(@This(), .{
        .base_index = codecs.Int(u8),
        .data = codecs.RemainingBytes,
    });
};

pub const FillRxBufferLong = struct {
    base_index: u16,
    data: []const u8,

    pub const Codec = codecs.Struct(@This(), .{
        .base_index = codecs.Int(u16),
        .data = codecs.RemainingBytes,
    });
};

pub const ProcessRxBuffer = struct {
    sender_id: u8,
    send_mode: BufferSendMode,
    length: u16,
    crc: u16,

    pub const Codec = codecs.Struct(@This(), .{
        .sender_id = codecs.Int(u8),
        .send_mode = BufferSendMode.Codec,
        .length = codecs.Int(u16),
        .crc = codecs.Int(u16),
    });
};

pub fn parsePacketBody(T: type, data: []const u8, diag: ?*codecs.DecodeDiagnostic) codecs.DecodeError!T {
    var reader = BufReader.from(data);
    return try T.Codec.decode(&reader, diag);
}

pub fn parsePacket(packet_type: PacketType, data: []const u8, diag: ?*codecs.DecodeDiagnostic) codecs.DecodeError!Packet {
    const info = @typeInfo(Packet).@"union";
    inline for (info.field_names, info.field_types) |field_name, field_type| {
        if (packet_type == @field(PacketType, field_name)) {
            return @unionInit(Packet, field_name, try parsePacketBody(field_type, data, diag));
        }
    }
    unreachable;
}

pub const FrameParseError = codecs.DecodeError || error{ NonVescFrame, UnknownPacket };
pub fn parseFrame(frame: CanFrame, diag: ?*codecs.DecodeDiagnostic) FrameParseError!struct {
    source_vesc_id: u8,
    packet: Packet,
} {
    const vesc_id = VescCanId.fromCanId(frame.id) catch |err| {
        switch (err) {
            error.NonVescId => return error.NonVescFrame,
        }
    };
    const packet_type = std.enums.fromInt(PacketType, vesc_id.packet_type) orelse return error.UnknownPacket;
    const packet = try parsePacket(packet_type, frame.data[0..@as(usize, frame.len)], diag);

    return .{
        .source_vesc_id = vesc_id.vesc_id,
        .packet = packet,
    };
}

pub const SerializeError = error{PacketTooLarge};

pub const PacketData = struct {
    data: [8]u8,
    len: u8,
};

pub fn serializePacketBody(packet: anytype) SerializeError!PacketData {
    var data: PacketData = undefined;
    var writer = codecs.BufWriter.into(&data.data);
    @TypeOf(packet).Codec.encode(packet, &writer) catch |err| {
        switch (err) {
            error.BufferTooSmall => return error.PacketTooLarge,
        }
    };
    data.len = @intCast(writer.writtenLength());

    return data;
}

pub fn serializeIntoFrame(dest_vesc_id: u8, packet: Packet) SerializeError!CanFrame {
    switch (packet) {
        inline else => |body, packet_type| {
            const serialized = try serializePacketBody(body);
            return .{
                .id = (VescCanId{ .vesc_id = dest_vesc_id, .packet_type = @backingInt(packet_type) }).toCanId(),
                .len = serialized.len,
                .data = serialized.data,
            };
        },
    }
}

test "Status1 parses negative values" {
    // erpm -1000, current -12.3 A, duty -0.5
    const id: VescCanId = .{ .vesc_id = 1, .packet_type = @backingInt(PacketType.status_1) };
    var frame: CanFrame = .{ .id = id.toCanId(), .len = 8 };
    frame.data = .{ 0xff, 0xff, 0xfc, 0x18, 0xff, 0x85, 0xfe, 0x0c };

    const id_and_packet = try parseFrame(frame, null);
    try std.testing.expectEqual(id_and_packet.source_vesc_id, 1);
    const packet = id_and_packet.packet;
    const status = switch (packet) {
        .status_1 => |status| status,
        else => return error.TestUnexpectedResult,
    };

    try std.testing.expectEqual(@as(f32, -1000), status.erpm);
    try std.testing.expectApproxEqAbs(@as(f32, -12.3), status.current, 0.001);
    try std.testing.expectApproxEqAbs(@as(f32, -0.5), status.duty_cycle, 0.001);
}

test "Status5 parses negative tachometer" {
    // tachometer -600 counts (-100 erev), 48.0 V
    const data = [_]u8{ 0xff, 0xff, 0xfd, 0xa8, 0x01, 0xe0, 0x00, 0x00 };
    const status = try parsePacketBody(Status5, &data, null);

    try std.testing.expectEqual(@as(f32, -100), status.tachometer);
    try std.testing.expectApproxEqAbs(@as(f32, 48.0), status.volts_in, 0.001);
}

test "NotifyBoot parses hardware name" {
    const data = [_]u8{ '6', '0', '_', 'M', 'K', '5' };
    const boot = try parsePacketBody(NotifyBoot, &data, null);

    try std.testing.expectEqualStrings("60_MK5", boot.hw_name);
}

test "Incomplete Status1 fails to parse" {
    const data = [_]u8{0x67};
    const result = parsePacketBody(Status1, &data, null);

    try std.testing.expectError(error.DecodeFailed, result);
}

test "FillRxBufferLong encoding" {
    const data = [_]u8{ 1, 2, 3, 4, 5, 6 };
    const packet = FillRxBufferLong{ .base_index = 0x3012, .data = &data };

    const expected = [_]u8{ 0x30, 0x12, 1, 2, 3, 4, 5, 6 };
    const encoded = try serializePacketBody(packet);
    try std.testing.expectEqualSlices(u8, &expected, encoded.data[0..encoded.len]);
}

test "FillRxBufferLong decoding" {
    const encoded = [_]u8{ 0x30, 0x12, 1, 2, 3, 4, 5, 6 };
    const packet = try parsePacketBody(FillRxBufferLong, &encoded, null);

    const expected_data = [_]u8{ 1, 2, 3, 4, 5, 6 };
    try std.testing.expectEqual(0x3012, packet.base_index);
    try std.testing.expectEqualSlices(u8, &expected_data, packet.data);
}

test "ProcessShortBuffer encoding" {
    const data = [_]u8{ 'h', 'e', 'l', 'l', 'o' };
    const packet = ProcessShortBuffer{ .sender_id = 0x12, .send_mode = .request, .data = &data };

    const expected = [_]u8{ 0x12, 0x00, 'h', 'e', 'l', 'l', 'o' };
    const encoded = try serializePacketBody(packet);
    try std.testing.expectEqualSlices(u8, &expected, encoded.data[0..encoded.len]);
}

test "ProcessShortBuffer decoding" {
    const encoded = [_]u8{ 0x12, 0x00, 'h', 'e', 'l', 'l', 'o' };
    const packet = try parsePacketBody(ProcessShortBuffer, &encoded, null);

    const expected_data = [_]u8{ 'h', 'e', 'l', 'l', 'o' };
    try std.testing.expectEqual(0x12, packet.sender_id);
    try std.testing.expectEqual(BufferSendMode.request, packet.send_mode);
    try std.testing.expectEqualSlices(u8, &expected_data, packet.data);
}

test "Pong decode with hardware type" {
    const encoded = [_]u8{ 0x23, 0x02 };
    const packet = try parsePacketBody(Pong, &encoded, null);

    try std.testing.expectEqual(0x23, packet.sender_id);
    try std.testing.expectEqual(.custom_module, packet.hardware_type);
}

test "Pong decode without hardware type" {
    const encoded = [_]u8{0x23};
    const packet = try parsePacketBody(Pong, &encoded, null);

    try std.testing.expectEqual(0x23, packet.sender_id);
    try std.testing.expectEqual(null, packet.hardware_type);
}
