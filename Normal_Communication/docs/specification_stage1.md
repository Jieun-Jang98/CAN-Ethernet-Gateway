# Stage 1 specification extraction

Source: *CAN-Ethernet bidirectional gateway normal communication final plan v3 - coding baseline*.

## Confirmed values

| Area | Confirmed value |
|---|---|
| CAN cycle | 20 ms |
| CAN frames | WHL_SPD11 `0x386/8`, SAS11 `0x2B0/5`, MDPS12 `0x251/8`, LKAS11 `0x340/8` |
| CAN byte order | DBC signals are little-endian |
| Wheel speed | bits 0/16, 14-bit unsigned, scale 0.03125 km/h |
| Steering angle | bit 0, 16-bit signed, scale 0.1 deg |
| Driver torque | bit 0, 11-bit unsigned, offset -1024 |
| LKAS torque / enable | bits 16/27, 11-bit unsigned offset -1024 / 1-bit |
| CAN counters | WHL split bits 14/30 (4-bit); SAS bit 32 (4-bit); MDPS bit 16 (8-bit); LKAS bit 36 (4-bit) |
| UDP byte order | Big-endian, explicit byte packing, no direct structure transmission |
| Vehicle_State | ID `0x10`, 12 bytes total, payload length 8 |
| Steering_Command | ID `0x20`, 8 bytes total, payload length 4 |
| UDP sequence | Independent per direction, uint16, starts at 0 and wraps after 65535 |
| Timeout | 100 ms or more since last valid receive |
| Rate limit | Away from zero 3; toward/crossing zero 7 per CAN frame |
| Rate reference | Last torque successfully registered in the CAN transmit queue |
| Rate violation | Clamp and report `RATE_CLAMP`; never latch for this reason alone |
| Torque limits | 20-<30 km/h: 100; 30-<80: 60; 80-120 inclusive: 30; otherwise inactive |
| Driver override | `abs(DriverTq)>150`; override at the sixth over-threshold sample; release when counter <=5 |
| Gateway rearm | Wait for disabled/zero or enabled `abs(torque)<=3` |
| LKAS CRC | poly `0x1D`, register init `0x22`, xor-out `0xDF`, MSB-first; input B0-B5 and B7 only |
| WHL checksum | popcount after excluding the upper two bits of B1/B3/B5/B7, then `(count XOR 9) & 0x0F` |

The pure rate function takes `last_can_torque` as an argument. It does not own
or update that state, so a later Gateway implementation can update it only
after successful CAN queue registration as required by sections 6.3 and 6.4.

## Conflict requiring document clarification

Section 5.2 says that the first implementation has no UDP application-layer
CRC. Section 11.3 nevertheless lists an invalid UDP "CRC/counter" test. Stage 1
does not invent a UDP CRC. It treats this as a mixed reference to UDP
length/reserved/sequence checks and CAN checksum/counter checks.

## Deferred by the requested scope

`pending_command`, `REARM_WAIT`, timeouts/recovery/latching state machines,
driver override state, real SocketCAN/UDP I/O, STM32 HAL, and complete Gateway
or ADAS programs are intentionally not implemented in Stage 1. Their constants
and pure helpers needed by later stages are provided.


## Implementation language

The common library and both test programs use C++17. Headers use .hpp and implementation files use .cpp; public functions and data types belong to namespace kuse. Wire formats and calculation rules remain unchanged.

