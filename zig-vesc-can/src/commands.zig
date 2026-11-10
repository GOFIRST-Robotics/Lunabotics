const std = @import("std");

const codec = @import("codec.zig");
const vesc_config = @import("vesc_config.zig");
const MotorControlConfig = vesc_config.MotorControlConfig;

// Requests

pub const RequestType = enum(u8) {
    get_mcconf = 14,
};

pub const Request = union(RequestType) {
    get_mcconf,

    pub const schema = .{
        .get_mcconf = codec.empty,
    };
};

// Responses

pub const ResponseType = enum(u8) {
    mcconf = 14,
};

pub const Response = union(ResponseType) {
    mcconf: MotorControlConfig,

    pub const schema = .{
        .mcconf = codec.structSchema(MotorControlConfig.schema),
    };
};
