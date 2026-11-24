const std = @import("std");

const zig_vesc_can = @import("zig-vesc-can");
const can = zig_vesc_can.can;
const commands = zig_vesc_can.commands;
const linux_can = zig_vesc_can.linux_can;

const motor_id: u8 = 1;
const status_frame_interval = linux_can.bcm_timeval{ .tv_sec = 0, .tv_usec = 20_000 }; // 20ms

const firmware_info: commands.FirmwareVersionInfo = .{
    .firmware_version_major = 1,
    .firmware_version_minor = 2,
    .hardware_name = "simulation",
    .stm32_uid = .{ 0, 1, 2, 3, 4, 5, 6, 7, 8, 9, 0, 1 },
    .pairing_done = false,
    .firmware_test_version_number = 0,
    .hardware_type = .vesc,
    .has_custom_config = false,
    .has_phase_filters = false,
    .qmlui_hardware_mode = .none,
    .qmlui_app_mode = .none,
    .qml_code_flags = 0,
    .nrf_flags = 0,
    .firmware_name = "zig",
    .hardware_crc = 12345,
};

pub fn main(init: std.process.Init) !void {
    const io = init.io;

    const socket = try can.openSocket("can0", null);
    defer can.closeSocket(socket);

    // Send status frames
    _ = try can.setupRepeatingSend(io, socket, .{
        .vesc_id = motor_id,
        .packet = .{ .status_1 = .{
            .erpm = 0.0,
            .current = 0.0,
            .duty_cycle = 0.0,
        } },
        .interval = status_frame_interval,
    }, null);

    // Listen for incoming packets
    _ = try can.setupReceive(io, socket, .{
        .vesc_id = motor_id,
        .packet_type = .ping,
        .ignore_duplicates = false,
        .timeout = null,
        .rate_limit = null,
    }, null);
    var command_receiver = try commands.CommandReceiver.init(io, socket, motor_id);

    std.debug.print("running simulated VESC motor on CAN id {d}\n", .{motor_id});

    // Receive events
    while (true) {
        var read_buf: can.ReadBuffer = undefined;
        const event = try can.readNextEvent(io, socket, &read_buf, null);

        switch (event.data) {
            .packet_received => |packet| switch (packet) {
                .ping => |ping| {
                    try can.sendOnce(io, socket, ping.sender_id, .{ .pong = .{
                        .sender_id = motor_id,
                        .hardware_type = .vesc,
                    } }, null);
                    std.debug.print("replying to ping from ID {d}\n", .{ping.sender_id});
                },
                else => {
                    if (try command_receiver.processPacket(packet)) |cmd| {
                        std.debug.print("received command: {any}\n", .{cmd});
                        switch (cmd) {
                            .request => |request| switch (request.body) {
                                .get_fw_version => {
                                    try commands.sendResponse(io, socket, .{
                                        .sender_id = motor_id,
                                        .receiver_id = request.sender_id,
                                        .body = .{ .fw_version = firmware_info },
                                    }, null);
                                },
                            },
                            .response => {},
                        }
                    }
                },
            },
            .packet_timeout => {},
        }
    }
}
