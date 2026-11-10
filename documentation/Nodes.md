# Overview
This document outlines what is necessary to define a node and how they communicate.

## Requirements

- __pub const NodeName: []const u8 = "MyNode";__
    This name is required so that meta functions in src/MFR/creation.zig can
    identify a node. Using @typeName is __NOT__ allowed in our meta programming.
    Zig developers have warned against using it in comptime other than for
    logging.

- __pub const inputType = struct{input1: \*i32, ...};__
    This type defines the inputs to the node's update function. All fields in
    it must be pointer types.
 
- __pub const outputConfigs = [\_]creation.OutputConfig{
        .{
            .output_node = OtherNode,
            .output_field = &.{.{
                .field = .other_input1,
                .buffering_type = .simple,
            }},
        },
    };__
    Output configs tell src/MFR/creation.zig what is expected for communication
    between nodes. The `.field` must be the name of a field in the __inputType__
    of OtherNode. The buffering type is the way in which the data is stored and
    transferred between nodes.

- __pub const outputType = creation.CreateOutputType(&outputConfigs);__
    creation will parse the outputConfigs and figure out the __outputType__ for
    the node's update function.

- __init(self: \*@This(), ...) void__
    An init function may be defined with its first argument being a pointer to
    itself. The init function does not return an instance of itself as the node
    has already been put on the stack. The init function should just initialize
    the node's state.

- __deinit(self: \*@This(), ...) void__
    If init allocates memory, opens a file descriptor or another operation that
    needs to be cleaned up, a deinit function should defined.

- __update(self: \*@This(), input: \*inputType, output: \*outputType) void__
    All nodes will have an update function with this signature. Depending on how
    the node is scheduled it can either be fully synchronous or asynchronous.
    This means that a node may use blocking Io as long as it is scheduled
    asynchronously.


## Implamentation

There are 4 different types found in main that hold the relevant data around
nodes. That is __NodesType__, __TransferBuffer__, __InputsType__, and 
__OutputsType__.

### RobotNodes
This is an array of node types that the robot will use.
For example
```zig
const RobotNodes: []const type = &.{
    ServerNode,
    CanIn,
    TeleopStateMachineNode,
};
```

#### NodesType
This struct is defined by 
`const NodesType = creation.CreateNodesType(RobotNodes);`
This is a struct containing each node struct.
It looks something like this:
```zig
const NodesType = struct {
    ServerNode: ServerNode,
    CanIn: CanIn,
    StateMachine: StateMachine,
    ...
}
```
The names correspond to the __NodeName__ defined in each node.

#### InputsType
This struct is defined by 
`const InputsType: type = creation.CreateInputs(RobotNodes);`
This is a struct containing each node's __inputType__ struct.
It looks something like this:
```zig
const InputsType = struct {
    ServerNode: ServerNode.inputType,
    CanIn: CanIn.inputType,
    StateMachine: StateMachine.inputType,
    ...
}
```

#### OutputsType
This struct is defined by 
`const OutputsType: type = creation.CreateOutputs(RobotNodes);`
This is a struct containing each node's __OutputsType__ struct.
It looks something like this:
```zig
const OutputsType = struct {
    ServerNode: struct {
        StateMachine: {
                input1: *i32,
                input2: *Timestamp,
            }
        },
    CanIn: struct{
        StateMachine: {
                input3: *usize,
            }
        },
    StateMachine: struct{}, // no output
    ...
}
```
While not shown here a node may have multiple outputs.

#### TransferBuffer
This struct is defined by
`const TransferBuffer: type = creation.CreateTransferBuffer(RobotNodes);`
This is a struct that actually contains the data being transferred between nodes.
It looks quit similar to output OutputsType but it actually holds that data
rather than just pointers to it.
```zig
const TransferBuffer = struct {
    ServerNode: struct {
        StateMachine: {
                input1: i32,
                input2: Timestamp,
            }
        },
    CanIn: struct{
        StateMachine: {
                input3: usize,
            }
        },
    StateMachine: struct{}, // no output
    ...
}
```

### Different Buffering Methods
When two synchronous nodes are communicating they can use a simple buffering
strategy because there they are operating on the same logical thread.
However if two nodes are running asynchronously data races need to be
considered. Recall the outputConfigs definition array. One of the options is
__buffering_type__. Found in creation.OutputBufferingMethod, this enum defines
how the robot should set up its buffer. If `.simple` then the buffer is just
an instance of the type defined in __inputType__ of the receiving node.
Another type defined is `.double_buffered`. This provides a struct with logic
that allows asynchronous nodes to transfer data involving a mutex to ensure
correctness.

#### Requirements for Custom Buffering Types
Custom buffering types should be generic to support any type. For example
a double buffered method is created with
`pub fn CreateDoubleBuffered(T: type) type`
`T` will be used to refer to the generic type in question.

- __pub fn init(self: \*@This()) void__
    Set up internal state such as a mutex.

- __pub fn getInputPtr(self: \*@This()) \*T__
    Give a pointer for the input. This will be put in __InputsType__.

- __pub fn getOutputPtr(self: \*@This()) \*T__
    Give a pointer for the output. This will be put in __OutputsType__.

- __pub fn preProducerUpdate(self: \*@This(), io: Io) Io.Cancelable!void__
    This function is run before the producer node's update function.

- __pub fn postProducerUpdate(self: \*@This(), io: Io) Io.Cancelable!void__
    This function is run after the producer node's update function.
    
- __pub fn preConsumerUpdate(self: \*@This(), io: Io) Io.Cancelable!void__
    This function is run before the consumer node's update function.

- __pub fn postConsumerUpdate(self: \*@This(), io: Io) Io.Cancelable!void__
    This function is run after the consumer node's update function.

