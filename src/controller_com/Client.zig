const std = @import("std");
const Io = std.Io;
const net = Io.net;
const linux = std.os.linux;
const log = std.log;
const input_devices = @import("input_devices.zig");
const protocol = @import("protocol.zig");
const ControllerData = protocol.ControllerData;
const StreamDeckData = protocol.StreamDeckData;

// a controller being unplugged is not handled properly yet by zig so this is here to shut it up
// otherwise it will give a useless stack trace because an errno is not being handled correctly
pub const std_options: std.Options = .{ .unexpected_error_tracing = false };

pub fn main(init: std.process.Init) !void {
    if (init.minimal.args.vector.len != 2) {
        log.err("The ip address or ssh config name of the robot must be provided, (e.g. '10.0.0.93' or 'jetson')", .{});
        return;
    }

    var custom_threaded: Io.Threaded = .init(init.gpa, .{});
    defer custom_threaded.deinit();
    const custom_io = custom_threaded.io();

    const server_address: net.IpAddress = determine_address: {
        const attempt_parse = net.IpAddress.parse(std.mem.span(init.minimal.args.vector[1]), protocol.port);
        if (attempt_parse) |parsed| {
            break :determine_address parsed;
        } else |_| {
            // this is running `eval "ssh -G pika | grep -i '^hostname' | awk '{print \$2}'"`
            const command = try std.fmt.allocPrint(init.gpa, "ssh -G {s} | grep -i '^hostname' | awk '{{print $2}}'", .{init.minimal.args.vector[1]});
            defer init.gpa.free(command);
            var get_ssh_ip = std.process.spawn(init.io, .{ .argv = &.{ "/bin/bash", "-c", command }, .stdout = .pipe }) catch |err| {
                log.err("Failed to spawn ssh command to resolve provided name", .{});
                return err;
            };

            var ipaddr: [1024]u8 = @splat(0);
            // substract one to account for newline
            const len = try get_ssh_ip.stdout.?.readStreaming(init.io, &.{&ipaddr}) - 1;
            _ = get_ssh_ip.wait(init.io) catch |err| {
                log.err("Faild to run ssh command to resolve provided name", .{});
                return err;
            };

            const parsed_ssh_ip = net.IpAddress.parse(ipaddr[0..len], protocol.port) catch |err| {
                log.err("Failed to parse ip address or ssh config name '{s}'", .{init.minimal.args.vector[1]});
                return err;
            };
            break :determine_address parsed_ssh_ip;
        }
    };

    const socket = get_sock: {
        // This is done because UDP is connectionless and only needs the socket
        const connection = server_address.connect(init.io, .{ .protocol = .udp, .mode = .dgram }) catch |err| {
            log.err("Failed to connect to {f} over UDP; {}", .{ server_address.ip4, err });
            return err;
        };
        break :get_sock connection.socket;
    };
    defer socket.close(init.io);

    log.info("Bound to {f}", .{server_address.ip4});

    var logitech_controller = try input_devices.findDevice(init.io, input_devices.logitech_name);
    defer {
        if (logitech_controller) |*con| {
            con.close(init.io);
        }
    }

    var logitech_controller_data: ControllerData = .{};
    var logitech_last_read_time: struct { sec: isize, nsec: isize } = undefined;

    var stream_deck = try input_devices.findDevice(init.io, input_devices.stream_deck_name);
    defer {
        if (stream_deck) |*sd| {
            sd.close(init.io);
        }
    }

    var stream_deck_data: StreamDeckData = .{};
    var stream_deck_last_read_time: struct { sec: isize, nsec: isize } = undefined;

    var read_logitech: Io.Future(@typeInfo(@TypeOf(input_devices.readFromLogitechController)).@"fn".return_type.?) = undefined;
    var find_logitech: Io.Future(@typeInfo(@TypeOf(input_devices.findDevice)).@"fn".return_type.?) = undefined;

    var read_stream_deck: Io.Future(@typeInfo(@TypeOf(input_devices.readFromStreamDeck)).@"fn".return_type.?) = undefined;
    var find_stream_deck: Io.Future(@typeInfo(@TypeOf(input_devices.findDevice)).@"fn".return_type.?) = undefined;

    while (true) {
        // =====================================================================
        // Either Search for the devices or get their input data
        // =====================================================================
        if (logitech_controller) |con| {
            read_logitech = try custom_io.concurrent(input_devices.readFromLogitechController, .{ custom_io, con, logitech_controller_data });
        } else {
            find_logitech = try custom_io.concurrent(input_devices.findDevice, .{ custom_io, input_devices.logitech_name });
        }

        if (stream_deck) |deck| {
            read_stream_deck = try custom_io.concurrent(input_devices.readFromStreamDeck, .{ custom_io, deck, stream_deck_data });
        } else {
            find_stream_deck = try custom_io.concurrent(input_devices.findDevice, .{ custom_io, input_devices.stream_deck_name });
        }

        try custom_io.sleep(.fromMilliseconds(20), .awake);

        // =====================================================================
        // Send Logitech Controller data or set new device file
        // =====================================================================
        if (logitech_controller) |_| {
            const read_from_logitech = read_logitech.cancel(custom_io);
            if (read_from_logitech) |v| {
                logitech_controller_data = v.data;
                logitech_last_read_time = .{ .sec = v.time.sec, .nsec = v.time.nsec };

                var dgram: [1024]u8 = undefined;
                var dgram_write_index: usize = 0;
                protocol.writeToDgram(protocol.ClientMessageType.controller, dgram[0..]);
                dgram_write_index += @sizeOf(protocol.ClientMessageType);

                protocol.writeToDgram(
                    protocol.TimeStamp{
                        .seconds = logitech_last_read_time.sec,
                        .nano_seconds = logitech_last_read_time.nsec,
                    },
                    dgram[dgram_write_index..],
                );
                dgram_write_index += @sizeOf(protocol.TimeStamp);

                protocol.writeToDgram(logitech_controller_data, dgram[dgram_write_index..]);
                dgram_write_index += @sizeOf(protocol.ControllerData);

                _ = socket.send(init.io, &server_address, dgram[0..dgram_write_index]) catch |err| {
                    log.warn("Got error while sending; {}", .{err});
                };
            } else |err| {
                switch (err) {
                    input_devices.ControllerReadError.LostController => {
                        logitech_controller.?.close(custom_io);
                        logitech_controller = null;
                    },
                    input_devices.ControllerReadError.ReadError => {
                        @panic("Unexpected read eror\n");
                    },
                }
            }
        } else {
            logitech_controller = find_logitech.cancel(custom_io) catch @panic("An error occured wile trying to find controller");
        }

        // =====================================================================
        // Send Stream Deck data or set new device file
        // =====================================================================
        if (stream_deck) |_| {
            const read_from_stream_deck = read_stream_deck.cancel(custom_io);
            if (read_from_stream_deck) |v| {
                stream_deck_data = v.data;
                stream_deck_last_read_time = .{ .sec = v.time.sec, .nsec = v.time.nsec };

                var dgram: [1024]u8 = undefined;
                var dgram_write_index: usize = 0;
                protocol.writeToDgram(protocol.ClientMessageType.stream_deck, &dgram);
                dgram_write_index += @sizeOf(protocol.ClientMessageType);

                protocol.writeToDgram(
                    protocol.TimeStamp{
                        .seconds = stream_deck_last_read_time.sec,
                        .nano_seconds = stream_deck_last_read_time.nsec,
                    },
                    dgram[dgram_write_index..],
                );
                dgram_write_index += @sizeOf(protocol.TimeStamp);

                protocol.writeToDgram(stream_deck_data, dgram[dgram_write_index..]);
                dgram_write_index += @sizeOf(protocol.StreamDeckData);

                _ = socket.send(init.io, &server_address, dgram[0..dgram_write_index]) catch |err| {
                    log.warn("Got error while sending; {}", .{err});
                };
            } else |err| {
                switch (err) {
                    input_devices.ControllerReadError.LostController => {
                        stream_deck.?.close(custom_io);
                        stream_deck = null;
                    },
                    input_devices.ControllerReadError.ReadError => {
                        @panic("Unexpected read eror\n");
                    },
                }
            }
        } else {
            stream_deck = find_stream_deck.cancel(custom_io) catch @panic("An error occured when trying to find the stream deck");
        }

        // =====================================================================
        // Print Info
        // =====================================================================
        var newlines_printed: u64 = 0;
        if (logitech_controller) |_| {
            var fmt_buf: [1024 * 3]u8 = undefined;
            const logitech_data_str = std.fmt.bufPrint(&fmt_buf, "{f}\n", .{logitech_controller_data}) catch fallback: {
                log.warn("Format buffer is too smaller", .{});
                break :fallback &.{};
            };
            newlines_printed += std.mem.count(u8, logitech_data_str, "\n");
            std.debug.print("{s}", .{logitech_data_str});
        } else {
            std.debug.print("No Logitech Controller Detected!\x1B[K\n", .{});
            newlines_printed += 1;
        }
        if (stream_deck) |_| {
            var fmt_buf: [1024 * 3]u8 = undefined;
            const stream_deck_data_str = std.fmt.bufPrint(&fmt_buf, "{f}\n", .{stream_deck_data}) catch fallback: {
                log.warn("Format buffer is too smaller", .{});
                break :fallback &.{};
            };
            newlines_printed += std.mem.count(u8, stream_deck_data_str, "\n");
            std.debug.print("{s}", .{stream_deck_data_str});
        } else {
            std.debug.print("No Stream Deck Detected!\x1B[K\n", .{});
            newlines_printed += 1;
        }
        std.debug.print("\x1B[0J\x1B[0K", .{});
        std.debug.print("\x1B[{d}F", .{newlines_printed});
    }
}
