const std = @import("std");
const socket_can = @import("socket_can.zig");
const can = @import("can.h");
const CanFrame = socket_can.CanFrame;
const bigToNative = std.mem.bigToNative;
const sliceAsBytes = std.mem.sliceAsBytes;
const linux = std.os.linux;

pub const CommandType = enum(u8) {
    SET_DUTY = 0,
    SET_CURRENT = 1,
    SET_CURRENT_BRAKE = 2,
    SET_RPM = 3,
    SET_POS = 4,
    FILL_RX_BUFFER = 5,
    FILL_RX_BUFFER_LONG = 6,
    PROCESS_RX_BUFFER = 7,
    PROCESS_SHORT_BUFFER = 8,
    STATUS = 9,
    SET_CURRENT_REL = 10,
    SET_CURRENT_BRAKE_REL = 11,
    SET_CURRENT_HANDBRAKE = 12,
    SET_CURRENT_HANDBRAKE_REL = 13,
    STATUS_2 = 14,
    STATUS_3 = 15,
    STATUS_4 = 16,
    PING = 17,
    PONG = 18,
    DETECT_APPLY_ALL_FOC = 19,
    DETECT_APPLY_ALL_FOC_RES = 20,
    CONF_CURRENT_LIMITS = 21,
    CONF_STORE_CURRENT_LIMITS = 22,
    CONF_CURRENT_LIMITS_IN = 23,
    CONF_STORE_CURRENT_LIMITS_IN = 24,
    CONF_FOC_ERPMS = 25,
    CONF_STORE_FOC_ERPMS = 26,
    STATUS_5 = 27,
    POLL_TS5700N8501_STATUS = 28,
    CONF_BATTERY_CUT = 29,
    CONF_STORE_BATTERY_CUT = 30,
    SHUTDOWN = 31,
    IO_BOARD_ADC_1_TO_4 = 32,
    IO_BOARD_ADC_5_TO_8 = 33,
    IO_BOARD_ADC_9_TO_12 = 34,
    IO_BOARD_DIGITAL_IN = 35,
    IO_BOARD_SET_OUTPUT_DIGITAL = 36,
    IO_BOARD_SET_OUTPUT_PWM = 37,
    BMS_V_TOT = 38,
    BMS_I = 39,
    BMS_AH_WH = 40,
    BMS_V_CELL = 41,
    BMS_BAL = 42,
    BMS_TEMPS = 43,
    BMS_HUM = 44,
    BMS_SOC_SOH_TEMP_STAT = 45,
    PSW_STAT = 46,
    PSW_SWITCH = 47,
    BMS_HW_DATA_1 = 48,
    BMS_HW_DATA_2 = 49,
    BMS_HW_DATA_3 = 50,
    BMS_HW_DATA_4 = 51,
    BMS_HW_DATA_5 = 52,
    BMS_AH_WH_CHG_TOTAL = 53,
    BMS_AH_WH_DIS_TOTAL = 54,
    UPDATE_PID_POS_OFFSET = 55,
    POLL_ROTOR_POS = 56,
    NOTIFY_BOOT = 57,
    STATUS_6 = 58,
    GNSS_TIME = 59,
    GNSS_LAT = 60,
    GNSS_LON = 61,
    GNSS_ALT_SPEED_HDOP = 62,
    UPDATE_BAUD = 63,
    BMS_STATUS_1 = 64,
    BMS_STATUS_2 = 65,
    BMS_STATUS_3 = 66,
    BMS_STATUS_4 = 67,
    BMS_STATUS_5 = 68,
};

fn vescParse(T: type, slice: []const u8, scalar: f32) f32 {
    std.debug.assert(@typeInfo(T).int.bits == 8 * slice.len);
    const underlying: T = std.mem.bytesToValue(T, slice);
    return @as(f32, @floatFromInt(bigToNative(T, underlying))) / scalar;
}

pub const StatusPacket1 = struct {
    erpm: f32,
    current: f32,
    duty_cycle: f32,
    pub fn parse(can_frame: CanFrame) @This() {
        return .{
            .erpm = vescParse(u32, can_frame.data.bytes[0..4], 1),
            .current = vescParse(u16, can_frame.data.bytes[4..6], 10),
            .duty_cycle = vescParse(u16, can_frame.data.bytes[6..], 1_000),
        };
    }
};

pub const StatusPacket2 = struct {
    amp_hours: f32,
    amp_hours_chg: f32,
    pub fn parse(can_frame: CanFrame) @This() {
        return .{
            .amp_hours = vescParse(u32, can_frame.data.bytes[0..4], 10_000),
            .amp_hours_chg = vescParse(u32, can_frame.data.bytes[4..], 10_000),
        };
    }
};

pub const StatusPacket3 = struct {
    watt_hours: f32,
    watt_hours_chg: f32,
};

pub const StatusPacket4 = struct {
    temp_FET: f32,
    temp_motor: f32,
    current_in: f32,
    PID_pos: f32,
};

pub const StatusPacket5 = struct {
    tachometer: f32,
    volts_in: f32,
    pub fn parse(can_frame: CanFrame) @This() {
        return .{
            .tachometer = vescParse(u32, can_frame.data.bytes[0..4], 6),
            .volts_in = vescParse(u16, can_frame.data.bytes[4..6], 10),
        };
    }
};

pub const StatusPacket6 = struct {
    ADC1: f32,
    ADC2: f32,
    ADC3: f32,
    PPM: f32,
};

pub fn vescWrite(T: type, value: f32, scalar: f32) T {
    return std.mem.nativeToBig(T, @trunc(value * scalar));
}

pub const SetCurrent = packed struct(u64) {
    current: i32, // 1_000
    _unused: u32 = 0,
};

pub const SetDuty = packed struct(u64) {
    duty: u32, // Scale of 100_000
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

pub fn WithBCMContext(T: type) type {
    return struct {
        value: T,
        timestamp: linux.kernel_timespec,
        is_alive: bool,
    };
}

pub const MotorStatus = struct {
    status1: ?WithBCMContext(StatusPacket1) = null,
    status2: ?WithBCMContext(StatusPacket2) = null,
    status3: ?WithBCMContext(StatusPacket3) = null,
    status4: ?WithBCMContext(StatusPacket4) = null,
    status5: ?WithBCMContext(StatusPacket5) = null,
    status6: ?WithBCMContext(StatusPacket6) = null,
};
