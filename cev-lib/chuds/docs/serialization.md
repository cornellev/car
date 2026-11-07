# CHUDS message bodies

A typed message body goes on the bus by copying the struct's raw bytes (`std::bit_cast`) and is
rebuilt byte-for-byte on the receiving node. So a body's exact in-memory representation _is_ the
wire format. That is fast and simple, but it means not every struct is safe to send.

## Requirements

`make_message<T>` / `body_as<T>` accept a body only if all of these hold. The `WireBody` concept
in `message.hpp` checks them.

1. Opt-in marker: the struct owns its CAN id, `static constexpr CanId kId{Band::..., sub};`.
2. Trivially copyable and standard-layout.
3. `sizeof(T) <= 64` (fits the CAN-FD payload, which is just the body).
4. No padding and no pointer members. Padding bytes are indeterminate and would leak onto the bus.
   The check builds `T{}` at compile time, so the body must have a constexpr default constructor.
   A plain struct of numbers always has one. A user-written constructor that is not `constexpr`
   does not count.

Plus two invariants every body must uphold, each backed by an assert:

- Fixed wire size. Each body carries `static_assert(sizeof(T) == <sum of field bytes>)`, so a
  change to a body's size fails to compile until the expected size is edited too.
- Little-endian host with IEEE 754 floats, asserted once in `message.hpp`
  (`std::endian::native == std::endian::little`, `std::numeric_limits<float>::is_iec559`).

## Why the marker

The compiler can check padding and pointers (requirement 4), but not "meant to go on the wire". A
stray `int` or an unrelated plain struct passes every other check.

So a body declares itself safe to send by declaring its `kId`. Anything without one, such as a
stray trivially-copyable type, is rejected at compile time.
The author owns the parts the compiler cannot see: fixed-width fields and no `bool`.

## What is not checked

`body_as<T>` copies whatever bytes arrived into `T` and does no field validation. The sender may be
buggy or built from an older struct, so any field can hold any byte. Callers check values
themselves.

- **Integers and floats:** every byte pattern is a valid value, so a bad value is only a wrong
  number. Range-check it where it matters.
- **Enums with a fixed underlying type:** every underlying value is allowed, including ones with no
  name. A `switch` over one needs a `default` that ignores unknown values.
- **`bool`:** only the bytes `0x00` and `0x01` are valid. Any other byte, such as `0x02`, is
  undefined behavior: the compiler may treat the same field as true in one `if` and false in the
  next. Never put a `bool` in a body. Use a `std::uint8_t` and test `!= 0`, or pack flags into a
  bitmask like `BodyState::bits`.

## Adding a body

1. Use only fixed-width integers (`std::uint16_t`, ...), IEEE floats, or fixed-underlying-type
   enums. No pointers, no `std::string`/`std::string_view`/`std::vector`/`std::span`, no
   `bool` (see above), and no bit-fields (their bit layout is up to the compiler).
2. Order fields so there is no padding, then `static_assert(sizeof(T) == N)`.
3. Add `static constexpr CanId kId{Band::..., sub};` inside the struct. It is `static`, so it
   takes up no bytes. Receivers read it with `catalog_body<T>(m)`.
4. Add the struct to `AllMessages` in `catalog.hpp`, so the id checks cover it.
