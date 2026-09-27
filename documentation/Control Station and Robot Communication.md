
To get controller inputs to the robot a custom protocol was made. This protocol uses UDP. Messages are just one way from the control station to the robot.

## Datagram Format

The format of every datagram is

| B0 - B1                | B1 - B17  | B17 Onward |
| ---------------------- | --------- | ---------- |
| ClientMessageType enum | TimeStamp | Data       |

The ClientMessageType enum specifies what the data will be. There is no length field because the robot code will be able to infer what the size of data. The TimeStamp struct has the seconds and nanoseconds defined in it. 

## UDP Considerations

UDP does **NOT** guarantee that messages make it to their destination, that they will be in order, or that they will only be received once. In our application it is very unlikely that there will be many of these faults because there is only one network node between the control station and robot. Despite this all code should be written assuming that any of those problems may occur.


## Protocol Implementation

All data should be converted to network order (big-endian). Use `std.mem.nativeToBig` and `std.mem.bigToNative` when writing and reading data respectively. When writing a struct to a datagram first `@bitCast` it to its backing integer, and then convert that integer to network order. `@bitCast` only works on a `packed struct`. When writing data to a datagram buffer do not hard code its backing integer. Use meta programming so that the size of the data's type may be changed later without requiring rewriting every time it is used. All `packed struct`s that will be sent across UDP must be divisible by 8 bits. This is a rule imposed by Zig due to how the `nativeToBig` and `bigToNative` functions are implemented. Use the helper functions in protocol.zig when reading and writing to a datagram buffer.

The UDP port used is 49153 but it could be any unused port.

## Code Considerations

Any functions or type definitions that are shared between the Client.zig and ServerNode.zig files should be put in protocol.zig. This prevents repeating yourself and makes compilation faster. Do **NOT** import the ServerNode.zig into Client.zig nor vise versa.