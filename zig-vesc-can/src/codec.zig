const std = @import("std");

pub const EncodeError = error{BufferTooSmall};
pub const DecodeError = error{DecodeFailed};

pub const DecodeDiagnostic = struct {
    message: []const u8 = "success",
    byte_position: usize = 0,
};

pub const boolean = Boolean{};
const Boolean = struct {
    pub fn encodedLength(_: @This(), value: anytype) usize {
        if (@TypeOf(value) != bool) @compileError("type must be bool");
        return 1;
    }

    pub fn encode(_: @This(), value: anytype, writer: *BufWriter) EncodeError!void {
        if (@TypeOf(value) != bool) @compileError("type must be bool");
        try writer.write(&.{if (value) 1 else 0});
    }

    pub fn decode(_: @This(), T: type, reader: *BufReader, diag: ?*DecodeDiagnostic) DecodeError!T {
        if (T != bool) @compileError("type must be bool");
        switch ((try reader.read(1, diag))[0]) {
            0 => return false,
            1 => return true,
            else => {
                reader.putDiagnostic("invalid boolean", diag);
                return error.DecodeFailed;
            },
        }
    }
};

pub const int = Int{};
const Int = struct {
    pub fn encodedLength(_: @This(), value: anytype) usize {
        const T = @TypeOf(value);
        if (@typeInfo(T) != .int) @compileError("type must be an integer");
        if (@bitSizeOf(T) % 8 != 0) @compileError("integer must be a multiple of 8 bits");
        return @sizeOf(T);
    }

    pub fn encode(_: @This(), value: anytype, writer: *BufWriter) EncodeError!void {
        const T = @TypeOf(value);
        if (@typeInfo(T) != .int) @compileError("type must be an integer");
        if (@bitSizeOf(T) % 8 != 0) @compileError("integer must be a multiple of 8 bits");

        const dest = try writer.writable(@sizeOf(T));
        std.mem.writeInt(T, dest[0..@sizeOf(T)], value, .big);
    }

    pub fn decode(_: @This(), T: type, reader: *BufReader, diag: ?*DecodeDiagnostic) DecodeError!T {
        if (@typeInfo(T) != .int) @compileError("type must be an integer");
        if (@bitSizeOf(T) % 8 != 0) @compileError("integer must be a multiple of 8 bits");

        const data = try reader.read(@sizeOf(T), diag);
        const raw: T = std.mem.bytesToValue(T, data);
        return std.mem.bigToNative(T, raw);
    }
};

pub const int_enum = IntEnum{};
const IntEnum = struct {
    pub fn encodedLength(_: @This(), value: anytype) usize {
        switch (@typeInfo(@TypeOf(value))) {
            .@"enum" => |e| return @sizeOf(e.tag_type),
            else => @compileError("type must be an enum"),
        }
    }

    pub fn encode(_: @This(), value: anytype, writer: *BufWriter) EncodeError!void {
        if (@typeInfo(@TypeOf(value)) != .@"enum") @compileError("type must be an enum");
        try int.encode(@intFromEnum(value), writer);
    }

    pub fn decode(_: @This(), T: type, reader: *BufReader, diag: ?*DecodeDiagnostic) DecodeError!T {
        const TagT = switch (@typeInfo(T)) {
            .@"enum" => |e| e.tag_type,
            else => @compileError("type must be an enum"),
        };

        if (std.enums.fromInt(T, try int.decode(TagT, reader, diag))) |v| {
            return v;
        } else {
            reader.putDiagnostic("invalid enum value", diag);
            return error.DecodeFailed;
        }
    }
};

pub const auto_f32 = AutoF32{};
const AutoF32 = struct {
    // TODO
};

pub const FixedFloat = struct {
    underlying: type,
    scalar: f32,

    // Most places use f64 scale factor, but some use f32 which gives very slightly different rounding
    use_f64_scale: bool = true,

    pub fn encodedLength(self: @This(), value: anytype) usize {
        _ = value;
        return @sizeOf(self.underlying);
    }

    pub fn encode(self: @This(), value: anytype, writer: *BufWriter) EncodeError!void {
        if (@TypeOf(value) != f32) @compileError("type must be f32");

        const underlying: self.underlying = if (self.use_f64_scale)
            @trunc(@as(f64, value) * self.scalar)
        else
            @trunc(value * @as(f32, self.scalar));

        try int.encode(underlying, writer);
    }

    pub fn decode(self: @This(), T: type, reader: *BufReader, diag: ?*DecodeDiagnostic) DecodeError!T {
        if (T != f32) @compileError("type must be f32");
        const underlying = try int.decode(self.underlying, reader, diag);
        return @as(f32, @floatFromInt(underlying)) / @as(f32, self.scalar);
    }
};

pub fn array(child_codec: anytype) struct {
    pub fn encodedLength(_: @This(), value: anytype) usize {
        if (@typeInfo(@TypeOf(value)) != .array) @compileError("type must be an array");

        var total_len: usize = 0;
        for (value) |elem| {
            total_len += child_codec.encodedLength(elem);
        }
        return total_len;
    }

    pub fn encode(_: @This(), value: anytype, writer: *BufWriter) EncodeError!void {
        if (@typeInfo(@TypeOf(value)) != .array) @compileError("type must be an array");
        for (value) |elem| {
            try child_codec.encode(elem, writer);
        }
    }

    pub fn decode(_: @This(), T: type, reader: *BufReader, diag: ?*DecodeDiagnostic) DecodeError!T {
        const info = switch (@typeInfo(T)) {
            .array => |i| i,
            else => @compileError("type must be an array"),
        };

        var value: [info.len]info.child = undefined;
        for (0..info.len) |i| {
            value[i] = try child_codec.decode(info.child, reader, diag);
        }
        return value;
    }
} {
    return .{};
}

fn validateStructSchema(T: type, schema: anytype) void {
    if (@typeInfo(T) != .@"struct") @compileError("type must be a struct");

    const SchemaT = @TypeOf(schema);
    inline for (std.meta.fields(T)) |field| {
        if (!@hasField(SchemaT, field.name)) {
            @compileError("missing schema for field `" ++ field.name ++ "`");
        }
    }
    inline for (std.meta.fields(SchemaT)) |field| {
        if (!@hasField(T, field.name)) {
            @compileError("no field found for schema entry `" ++ field.name ++ "`");
        }
    }
}

pub fn structSchema(schema: anytype) struct {
    pub fn encodedLength(_: @This(), value: anytype) usize {
        validateStructSchema(@TypeOf(value), schema);

        var total_len: usize = 0;
        inline for (std.meta.fields(@TypeOf(schema))) |field| {
            const field_codec = @field(schema, field.name);
            const field_value = @field(value, field.name);
            total_len += field_codec.encodedLength(field_value);
        }
        return total_len;
    }

    pub fn encode(_: @This(), value: anytype, writer: *BufWriter) EncodeError!void {
        const T = @TypeOf(value);
        validateStructSchema(T, schema);

        const SchemaT = @TypeOf(schema);
        inline for (std.meta.fields(SchemaT)) |field| {
            const field_codec = @field(schema, field.name);
            const field_value = @field(value, field.name);
            try field_codec.encode(field_value, writer);
        }
    }

    pub fn decode(_: @This(), T: type, reader: *BufReader, diag: ?*DecodeDiagnostic) DecodeError!T {
        validateStructSchema(T, schema);

        const SchemaT = @TypeOf(schema);
        var value: T = undefined;
        inline for (std.meta.fields(SchemaT)) |field| {
            const field_codec = @field(schema, field.name);
            const FieldType = @FieldType(T, field.name);
            @field(value, field.name) = try field_codec.decode(FieldType, reader, diag);
        }

        return value;
    }
} {
    return .{};
}

pub const remaining_bytes = RemainingBytes{};
const RemainingBytes = struct {
    pub fn encodedLength(_: @This(), value: anytype) usize {
        if (@TypeOf(value) != []const u8) @compileError("type must be []const u8");
        return value.len;
    }

    pub fn encode(_: @This(), value: anytype, writer: *BufWriter) EncodeError!void {
        if (@TypeOf(value) != []const u8) @compileError("type must be []const u8");
        try writer.write(value);
    }

    pub fn decode(_: @This(), T: type, reader: *BufReader, diag: ?*DecodeDiagnostic) DecodeError!T {
        if (T != []const u8) @compileError("type must be []const u8");
        return try reader.read(reader.remaining(), diag);
    }
};

pub const BufReader = struct {
    data: []const u8,
    pos: usize,

    pub fn from(data: []const u8) @This() {
        return .{ .data = data, .pos = 0 };
    }

    pub fn read(self: *@This(), count: usize, diag: ?*DecodeDiagnostic) DecodeError![]const u8 {
        if (count > self.remaining()) {
            self.putDiagnostic("incomplete data", diag);
            return error.DecodeFailed;
        } else {
            const slice = self.data[self.pos..][0..count];
            self.pos += count;
            return slice;
        }
    }

    pub fn putDiagnostic(self: @This(), message: []const u8, diag: ?*DecodeDiagnostic) void {
        if (diag) |d| {
            d.message = message;
            d.byte_position = self.pos;
        }
    }

    pub fn remaining(self: @This()) usize {
        return self.data.len - self.pos;
    }
};

pub const BufWriter = struct {
    dest: []u8,
    length: usize,

    pub fn into(dest: []u8) @This() {
        return .{ .dest = dest, .length = 0 };
    }

    pub fn write(self: *@This(), data: []const u8) EncodeError!void {
        const dest = try self.writable(data.len);
        @memcpy(dest, data);
    }

    pub fn writable(self: *@This(), len: usize) EncodeError![]u8 {
        if (self.length + len > self.dest.len) {
            return error.BufferTooSmall;
        } else {
            const slice = self.dest[self.length..][0..len];
            self.length += len;
            return slice;
        }
    }

    pub fn writtenData(self: @This()) []u8 {
        return self.dest[0..self.length];
    }

    pub fn writtenLength(self: @This()) usize {
        return self.length;
    }
};

test "codec" {
    const Cat = struct {
        adult: bool,
        leg_count: u16,
        fur_color: enum(u8) { black = 0, gray = 1, orange = 2 },
        weight: f32,
        ear_angles: [2]f32,
        name: []const u8,

        pub const schema = .{
            .adult = boolean,
            .leg_count = int,
            .fur_color = int_enum,
            .weight = FixedFloat{ .underlying = i16, .scalar = 100 },
            .ear_angles = array(FixedFloat{ .underlying = i8, .scalar = 10 }),
            .name = remaining_bytes,
        };
    };

    const original_cat = Cat{
        .adult = true,
        .leg_count = 4,
        .fur_color = .gray,
        .weight = 12.3,
        .ear_angles = .{ 10, 12 },
        .name = "kit",
    };
    const encoded_cat = [_]u8{ 0x01, 0x00, 0x04, 0x01, 0x04, 0xce, 0x64, 0x78, 'k', 'i', 't' };
    const cat_codec = structSchema(Cat.schema);

    try std.testing.expectEqual(11, cat_codec.encodedLength(original_cat));

    var buf: [11]u8 = undefined;
    var buf_writer = BufWriter.into(&buf);
    try cat_codec.encode(original_cat, &buf_writer);
    try std.testing.expectEqualSlices(u8, &encoded_cat, buf_writer.writtenData());

    var buf_reader = BufReader.from(&encoded_cat);
    const decoded_cat = try cat_codec.decode(Cat, &buf_reader, null);
    try std.testing.expectEqualDeep(original_cat, decoded_cat);
}
