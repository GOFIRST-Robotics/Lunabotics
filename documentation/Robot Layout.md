# Philosophy

- One way call graph
- Explicit configuration
- Modularity


## Nodes

The robot can be broken down into nodes. A node is a peice of state that can
either recieve input from other nodes and output to other nodes. The functions
found in a node's file are entierly encapsulated. No other node will directly
call another node's internal functions. Nodes may only comunicate with
eachother with data.

### Formal Requirements

A Node must have the following:
- An `inputType`
- An `outputType`
- An `update` function

Optionally, an `init` and `deinit` functions can be included but will need to
be called manually and will not be used in meta programming.

Here is a simple example of a node.

*In file MyNode.zig*
```zig
const OutputToNode = @import("../other_node/OtherNode.zig");
const MyNode = @This();

io: Io

pub const inputType = struct {
    dataOfInterest: i64,
};

pub const outputType = struct {
    other_node: OutputToNode.inputType,
};

pub fn init(io: Io) @This() {
    return .{
        .io = io,
    };
}

pub fn deinit(self: *@This()) void {
    _ = self;
}

pub fn update(self: *@This(), input: *const inputType, output: *outputType) void {
    if (input.dataOfInterest >= 0) {
        output.other_node.powerOn = true;
    } else {
        output.other_node.powerOn = false;
    }
}

```

First notice that this is the definition of an entire file as seen by the line
`const MyNode = @This()` where the file is named *MyNode.zig*. 
The update function's signiture will be the same in every node. The
`*outputType` argument is entierly owned by the update function. That is to 
say, you may write to it however you like. There will be no race conditions
or other consideration when interacting with it. However note that the data
inside of it might be garbage and should be not read.
`*const inputType` is const because the only purpose of the input is to read the
data.

In the pursuit of having a one way call graph and simple structure, the
definition of a node does not include a reference to the node that gives it
input. The `inputType` of a node can be thought of its interface. In that way
a node does not need to know the context of the caller only the caller needs to
know the context of the callee.
