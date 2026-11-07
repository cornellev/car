# CHUDS addressing

Status: draft, pending team ratification.

## Id layout

Every CHUDS frame uses an 11-bit standard CAN id. The top 3 bits are the message class, and the
low 8 bits are the subaddress. The lower id wins arbitration, so the class order is the priority
order.

| Class     | Value | Id range      | Subaddress means         |
| --------- | ----- | ------------- | ------------------------ |
| Emergency | `0`   | `0x000-0x0FF` | severity (0 most severe) |
| Command   | `1`   | `0x100-0x1FF` | recipient node id        |
| Telemetry | `2`   | `0x200-0x2FF` | source node block `0xNM` |
| Status    | `3`   | `0x300-0x3FF` | source node block `0xNM` |
| Reserved  | `4-7` | `0x400-0x7FF` | future classes           |

- Telemetry and Status subaddresses are node blocks `0xNM`: node N in the high nibble, its message
  M in the low nibble, so up to 16 messages per node. `catalog.hpp` checks at compile time that
  every such id sits in a registered node's block.
- The transports reject extended 29-bit, remote, and classic CAN frames (`ForeignFrame`).
- `decode` rejects reserved classes (`UnknownClass`) and lengths that are not a CAN-FD size
  (`BadLength`).
- The bus is CHUDS-only. There is no magic byte or header, so any 11-bit CAN-FD frame in a valid
  class decodes as CHUDS.

## Classes

- **Emergency:** broadcast, and every node with outputs acts on it. The subaddress is the
  severity, so `0x000`, the most severe STOP, wins every arbitration. A STOP body is
  `[sender][reason]`, where reason is a `StopReason`. Only the VCU sends STOP, because CAN allows
  one transmitter per id. A node reports a fault to the VCU instead. This is the software stop. The
  hardwired e-stop works without any bus.
- **Command:** from the VCU to the nodes. The subaddress is the recipient node id. A broadcast
  uses a subaddress no node owns. The broadcasts today are `BodyState` at `0x108` and
  `StatusBlink` at `0x109`.
- **Telemetry:** each node publishes under its own node block, and any consumer filters for it.

## Assigning subaddresses

Within a class, the lower subaddress always wins the bus, so a chatty low-address node can starve
a higher one. Assign subaddresses by priority, not convenience. The allocation is part of the
protocol: changing a node's subaddress means updating every node that uses it.

## Payload

The CAN-FD data field is just the body, up to 64 bytes, with no header. The id says what the body
is: each struct in `catalog.hpp` owns its id as `kId`, and struct bodies follow
[`serialization.md`](serialization.md).

- CAN-FD pads a frame up to the next valid size (0-8, 12, 16, 20, 24, 32, 48, 64 bytes), so a
  received body includes any padding.
- `body_as<T>` accepts a frame only if its id is `T::kId` and its length is the padded size of `T`.
- STOP has no `kId`, because its id varies with severity. `read_stop` matches the Emergency class
  with a 2-byte body.

## Examples

| Id      | Meaning                                                          |
| ------- | ---------------------------------------------------------------- |
| `0x000` | most severe STOP, every node with outputs goes to safe states    |
| `0x108` | `BodyState` broadcast: turn signals, headlights, horn, wiper     |
| `0x140` | command to node `0x40`, such as a throttle body                  |
| `0x210` | telemetry from node `0x10` (front_aux)                           |

## Open questions

- Multicast groups (the proposal's Universal Firmware Id / Group Id / Domain Specific Id) as
  sub-ranges of the Command class.
- A sender id in payloads other than STOP.
- A send-rate limit per node.
- How long a node may go without a drive command before it stops the motor. For comparison, a
  node that hears no `BodyState` for 1 s switches its lights, horn, and wiper to safe states.
