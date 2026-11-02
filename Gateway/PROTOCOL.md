# Gateway protocol

모든 다중 바이트 값과 CAN 신호는 Intel Little Endian이다. Checksum은 1차
시험에서 사용하지 않으며 해당 바이트는 0이다.

## CAN sensor input

| CAN ID | DBC message | Signal | Encoding |
|---|---|---|---|
| `0x386` | `WHL_SPD11` | `WHL_SPD_FL` | `0|14@1+`, factor `0.03125 km/h` |
| `0x386` | `WHL_SPD11` | `WHL_SPD_FR` | `16|14@1+`, factor `0.03125 km/h` |
| `0x2B0` | `SAS11` | `SAS_Angle` | `0|16@1-`, factor `0.1 deg` |
| `0x251` | `MDPS12` | `CR_Mdps_StrColTq` | `0|11@1+`, offset `-1024` |

## UDP vehicle state: Gateway to ADAS

Destination: `192.168.50.12:5000`, period: 20 ms, type: `0x10`, total 12 bytes.

| Byte | Field | Encoding |
|---:|---|---|
| 0 | Type | `0x10` |
| 1..2 | Sequence | `uint16`, wraps naturally |
| 3 | Payload length | `8` |
| 4..5 | Front-left speed | `uint16`, `0.01 km/h` |
| 6..7 | Front-right speed | `uint16`, `0.01 km/h` |
| 8..9 | Steering angle | `int16`, `0.1 deg` |
| 10..11 | Steering-column torque | `int16`, DBC physical value |

센서 세 종류 중 하나라도 수신되지 않았거나 200 ms보다 오래되면 이 패킷은
전송하지 않는다.

## UDP steering command: ADAS to Gateway

Destination: `192.168.50.11:5001`, type: `0x20`, total 10 bytes.

| Byte | Field | Encoding |
|---:|---|---|
| 0 | Type | `0x20` |
| 1..2 | Sequence | `uint16` |
| 3 | Payload length | `6` |
| 4..5 | Target steering angle | `int16`, `0.1 deg` |
| 6..7 | Torque limit | `uint16`, accepted range 0..200 |
| 8 | Enable | `0` or `1` |
| 9 | Reason | 0..15 |

The gateway converts this packet to custom CAN ID `0x340`, DLC 8:

| Signal | DBC-style definition |
|---|---|
| `TargetAngle` | `0|16@1- (0.1,0)` |
| `TorqueLimit` | `16|11@1+ (1,0)` |
| `ControlEnable` | `27|1@1+ (1,0)` |
| `ControlReason` | `28|4@1+ (1,0)` |
| `AliveCounter` | `36|4@1+ (1,0)` |
| `Checksum` | `48|8@1+ (1,0)`, fixed to 0 |

## UDP speed command: ADAS to Gateway

Destination: `192.168.50.11:5001`, type: `0x21`, total 10 bytes.

| Byte | Field | Encoding |
|---:|---|---|
| 0 | Type | `0x21` |
| 1..2 | Sequence | `uint16` |
| 3 | Payload length | `6` |
| 4..5 | Target speed | `uint16`, `0.01 km/h` |
| 6 | Deceleration | `uint8`, 0..100 percent |
| 7 | Enable | `0` or `1` |
| 8 | Reason | 0..15 |
| 9 | Reserved | must be 0 |

The gateway converts this packet to custom CAN ID `0x341`, DLC 8:

| Signal | DBC-style definition |
|---|---|
| `TargetSpeed` | `0|14@1+ (0.03125,0)` |
| `DecelerationPercent` | `16|8@1+ (1,0)` |
| `ControlEnable` | `24|1@1+ (1,0)` |
| `ControlReason` | `25|4@1+ (1,0)` |
| `AliveCounter` | `32|4@1+ (1,0)` |
| `Checksum` | `48|8@1+ (1,0)`, fixed to 0 |

유효한 ADAS 명령이 100 ms 동안 들어오지 않으면 새 CAN 제어 프레임을 보내지
않고 터미널에 타임아웃을 출력한다.

