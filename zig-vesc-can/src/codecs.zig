const std = @import("std");

pub const EncodeError = error{BufferTooSmall};
pub const DecodeError = error{DecodeFailed};

pub const DecodeDiagnostic = struct {
    message: []const u8 = "success",
    byte_position: usize = 0,
};

pub const Empty = struct {
    pub const Value = void;

    pub fn encodedLength(_: Value) usize {
        return 0;
    }

    pub fn encode(_: Value, _: *BufWriter) EncodeError!void {}

    pub fn decode(_: *BufReader, _: ?*DecodeDiagnostic) DecodeError!Value {
        return {};
    }
};

pub const Boolean = struct {
    pub const Value = bool;

    pub fn encodedLength(_: Value) usize {
        return 1;
    }

    pub fn encode(value: Value, writer: *BufWriter) EncodeError!void {
        try writer.write(&.{if (value) 1 else 0});
    }

    pub fn decode(reader: *BufReader, diag: ?*DecodeDiagnostic) DecodeError!Value {
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

pub fn Int(T: type) type {
    if (@typeInfo(T) != .int) @compileError("type must be an integer");
    if (@bitSizeOf(T) % 8 != 0) @compileError("integer must have a multiple of 8 bits");

    return struct {
        pub const Value = T;

        pub fn encodedLength(_: Value) usize {
            return @sizeOf(T);
        }

        pub fn encode(value: Value, writer: *BufWriter) EncodeError!void {
            const dest = try writer.writable(@sizeOf(T));
            std.mem.writeInt(T, dest[0..@sizeOf(T)], value, .big);
        }

        pub fn decode(reader: *BufReader, diag: ?*DecodeDiagnostic) DecodeError!Value {
            const data = try reader.read(@sizeOf(T), diag);
            const raw = std.mem.bytesToValue(T, data);
            return std.mem.bigToNative(T, raw);
        }
    };
}

pub fn IntEnum(T: type) type {
    const tag_type = switch (@typeInfo(T)) {
        .@"enum" => |e| e.tag_type,
        else => @compileError("type must be an enum"),
    };
    const TagCodec = Int(tag_type);

    return struct {
        pub const Value = T;

        pub fn encodedLength(value: Value) usize {
            return TagCodec.encodedLength(@backingInt(value));
        }

        pub fn encode(value: Value, writer: *BufWriter) EncodeError!void {
            try TagCodec.encode(@backingInt(value), writer);
        }

        pub fn decode(reader: *BufReader, diag: ?*DecodeDiagnostic) DecodeError!Value {
            const tag = try TagCodec.decode(reader, diag);
            if (std.enums.fromInt(T, tag)) |v| {
                return v;
            } else {
                reader.putDiagnostic("invalid enum value", diag);
                return error.DecodeFailed;
            }
        }
    };
}

pub fn FixedFloat(Underlying: type, scalar: anytype) type {
    const UnderlyingCodec = Int(Underlying);

    return struct {
        pub const Value = f32;

        pub fn encodedLength(_: Value) usize {
            return @sizeOf(Underlying);
        }

        pub fn encode(value: Value, writer: *BufWriter) EncodeError!void {
            const underlying: Underlying = @trunc(value * scalar);
            try UnderlyingCodec.encode(underlying, writer);
        }

        pub fn decode(reader: *BufReader, diag: ?*DecodeDiagnostic) DecodeError!Value {
            const underlying = try UnderlyingCodec.decode(reader, diag);
            return @as(f32, @floatFromInt(underlying)) / @as(f32, scalar);
        }
    };
}

pub fn Array(len: usize, ChildCodec: type) type {
    return struct {
        pub const Value = [len]ChildCodec.Value;

        pub fn encodedLength(value: Value) usize {
            var total_len: usize = 0;
            for (value) |elem| {
                total_len += ChildCodec.encodedLength(elem);
            }
            return total_len;
        }

        pub fn encode(value: Value, writer: *BufWriter) EncodeError!void {
            for (value) |elem| {
                try ChildCodec.encode(elem, writer);
            }
        }

        pub fn decode(reader: *BufReader, diag: ?*DecodeDiagnostic) DecodeError!Value {
            var value: Value = undefined;
            for (0..len) |i| {
                value[i] = try ChildCodec.decode(reader, diag);
            }
            return value;
        }
    };
}

// Reads as present only if the input buffer has remaining data
// Can only be read reliably if it is placed at the end of the buffer
pub fn Optional(ChildCodec: type) type {
    return struct {
        pub const Value = ?ChildCodec.Value;

        pub fn encodedLength(value: Value) usize {
            if (value) |v| {
                return ChildCodec.encodedLength(v);
            } else {
                return 0;
            }
        }

        pub fn encode(value: Value, writer: *BufWriter) EncodeError!void {
            if (value) |v| {
                try ChildCodec.encode(v, writer);
            }
        }

        pub fn decode(reader: *BufReader, diag: ?*DecodeDiagnostic) DecodeError!Value {
            if (reader.remaining() > 0) {
                return try ChildCodec.decode(reader, diag);
            } else {
                return null;
            }
        }
    };
}

// Serialization order of fields is determined by the order in the schema
pub fn Struct(T: type, schema: anytype) type {
    if (@typeInfo(T) != .@"struct") @compileError("type must be a struct");

    const SchemaT = @TypeOf(schema);
    if (@typeInfo(SchemaT) != .@"struct") @compileError("schema must be a struct");

    inline for (@typeInfo(T).@"struct".field_names) |field_name| {
        if (!@hasField(SchemaT, field_name)) {
            @compileError("missing schema for field `" ++ field_name ++ "`");
        }
    }
    inline for (@typeInfo(SchemaT).@"struct".field_names) |field_name| {
        if (!@hasField(T, field_name)) {
            @compileError("no field found for schema entry `" ++ field_name ++ "`");
        }
    }
    const field_names = @typeInfo(SchemaT).@"struct".field_names;

    return struct {
        pub const Value = T;

        pub fn encodedLength(value: Value) usize {
            var total_len: usize = 0;
            inline for (field_names) |field_name| {
                const field_codec = @field(schema, field_name);
                const field_value = @field(value, field_name);
                total_len += field_codec.encodedLength(field_value);
            }
            return total_len;
        }

        pub fn encode(value: Value, writer: *BufWriter) EncodeError!void {
            inline for (field_names) |field_name| {
                const field_codec = @field(schema, field_name);
                const field_value = @field(value, field_name);
                try field_codec.encode(field_value, writer);
            }
        }

        pub fn decode(reader: *BufReader, diag: ?*DecodeDiagnostic) DecodeError!Value {
            var value: Value = undefined;
            inline for (field_names) |field_name| {
                const field_codec = @field(schema, field_name);
                @field(value, field_name) = try field_codec.decode(reader, diag);
            }
            return value;
        }
    };
}

pub fn PrefixedUnion(T: type, TagCodec: type, schema: anytype) type {
    if (@typeInfo(T) != .@"union") @compileError("type must be a union");

    const SchemaT = @TypeOf(schema);
    if (@typeInfo(SchemaT) != .@"struct") @compileError("schema must be a struct");

    inline for (@typeInfo(T).@"union".field_names) |field_name| {
        if (!@hasField(SchemaT, field_name)) {
            @compileError("missing schema for field `" ++ field_name ++ "`");
        }
    }
    inline for (@typeInfo(SchemaT).@"struct".field_names) |field_name| {
        if (!@hasField(T, field_name)) {
            @compileError("no field found for schema entry `" ++ field_name ++ "`");
        }
    }
    const field_names = @typeInfo(SchemaT).@"struct".field_names;

    return struct {
        pub const Value = T;

        pub fn encodedLength(value: Value) usize {
            switch (value) {
                inline else => |content, tag| {
                    const prefix_len = TagCodec.encodedLength(tag);
                    const content_codec = @field(schema, @tagName(tag));
                    const content_len = content_codec.encodedLength(content);
                    return prefix_len + content_len;
                },
            }
        }

        pub fn encode(value: Value, writer: *BufWriter) EncodeError!void {
            switch (value) {
                inline else => |content, tag| {
                    try TagCodec.encode(tag, writer);
                    const content_codec = @field(schema, @tagName(tag));
                    try content_codec.encode(content, writer);
                },
            }
        }

        pub fn decode(reader: *BufReader, diag: ?*DecodeDiagnostic) DecodeError!Value {
            const TagT = std.meta.Tag(T);
            const tag = try TagCodec.decode(reader, diag);

            inline for (field_names) |field_name| {
                const field_tag = @field(TagT, field_name);
                if (tag == field_tag) {
                    const content_codec = @field(schema, field_name);
                    const value = try content_codec.decode(reader, diag);
                    return @unionInit(T, field_name, value);
                }
            }
            unreachable;
        }
    };
}

pub const NullTerminatedBytes = struct {
    pub const Value = []const u8;

    pub fn encodedLength(value: Value) usize {
        return value.len + 1;
    }

    pub fn encode(value: Value, writer: *BufWriter) EncodeError!void {
        try writer.write(value);
        try writer.write(&.{0});
    }

    pub fn decode(reader: *BufReader, diag: ?*DecodeDiagnostic) DecodeError!Value {
        const with_null = try reader.readUntil(0, diag);
        return with_null[0..(with_null.len - 1)];
    }
};

pub const RemainingBytes = struct {
    pub const Value = []const u8;

    pub fn encodedLength(value: Value) usize {
        return value.len;
    }

    pub fn encode(value: Value, writer: *BufWriter) EncodeError!void {
        try writer.write(value);
    }

    pub fn decode(reader: *BufReader, diag: ?*DecodeDiagnostic) DecodeError!Value {
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

    pub fn readUntil(self: *@This(), end_marker: u8, diag: ?*DecodeDiagnostic) DecodeError![]const u8 {
        const start = self.pos;

        while (self.pos < self.data.len) {
            if (self.data[self.pos] == end_marker) {
                self.pos += 1;
                return self.data[start..self.pos];
            }
            self.pos += 1;
        }

        self.putDiagnostic("incomplete data", diag);
        return error.DecodeFailed;
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
    const PatternType = enum(u8) { solid = 0, stripes = 1 };

    const Pattern = union(PatternType) {
        solid: void,
        stripes: u8,

        pub const Codec = PrefixedUnion(@This(), IntEnum(PatternType), .{
            .solid = Empty,
            .stripes = Int(u8),
        });
    };

    const FurColor = enum(u8) { black = 0, gray = 1, orange = 2 };

    const Cat = struct {
        adult: bool,
        leg_count: u16,
        fur_color: FurColor,
        pattern: Pattern,
        weight: f32,
        ear_angles: [2]f32,
        ghost: void,
        favorite_word: []const u8,
        name: []const u8,

        pub const Codec = Struct(@This(), .{
            .adult = Boolean,
            .leg_count = Int(u16),
            .fur_color = IntEnum(FurColor),
            .pattern = Pattern.Codec,
            .weight = FixedFloat(i16, @as(f64, 100)),
            .ear_angles = Array(2, FixedFloat(i8, @as(f64, 10))),
            .ghost = Empty,
            .favorite_word = NullTerminatedBytes,
            .name = RemainingBytes,
        });
    };

    const original_cat = Cat{
        .adult = true,
        .leg_count = 4,
        .fur_color = .gray,
        .pattern = .{ .stripes = 12 },
        .weight = 12.3,
        .ear_angles = .{ 10, 12 },
        .ghost = {},
        .favorite_word = "food",
        .name = "kit",
    };
    const encoded_cat = [_]u8{ 0x01, 0x00, 0x04, 0x01, 0x01, 0x0c, 0x04, 0xce, 0x64, 0x78, 'f', 'o', 'o', 'd', 0x00, 'k', 'i', 't' };

    try std.testing.expectEqual(encoded_cat.len, Cat.Codec.encodedLength(original_cat));

    var buf: [encoded_cat.len]u8 = undefined;
    var buf_writer = BufWriter.into(&buf);
    try Cat.Codec.encode(original_cat, &buf_writer);
    try std.testing.expectEqualSlices(u8, &encoded_cat, buf_writer.writtenData());

    var buf_reader = BufReader.from(&encoded_cat);
    const decoded_cat = try Cat.Codec.decode(&buf_reader, null);
    try std.testing.expectEqualDeep(original_cat, decoded_cat);
}
