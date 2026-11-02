# NUCLEO-F446RE ↔ Gateway 양방향 CAN 시험

이 프로젝트는 Raspberry Pi Gateway의 `Gateway/gateway_custom.dbc`와 동일한
비트 배치를 사용한다.

## 하드웨어 및 CAN 설정

- Board: NUCLEO-F446RE
- CAN1 RX: D15 / PB8
- CAN1 TX: D14 / PB9
- Classical CAN, Standard 11-bit ID, 500 kbit/s
- CAN peripheral clock: 42 MHz
- Bit timing: Prescaler 6, BS1 11 TQ, BS2 2 TQ, SJW 1 TQ

STM32의 PB8/PB9는 CAN_H/CAN_L 신호가 아니므로 CAN 트랜시버가 반드시
필요하다.

```text
PB9  (CAN1_TX) -> CAN transceiver TXD
PB8  (CAN1_RX) <- CAN transceiver RXD
Transceiver CAN_H <-> Raspberry Pi HAT CAN_H
Transceiver CAN_L <-> Raspberry Pi HAT CAN_L
GND               <-> GND
```

CAN 버스 양 끝에 각각 120-ohm 종단 저항을 둔다.

## STM32 → Gateway 센서 송신

세 프레임을 20 ms마다 송신한다. 모든 신호는 Intel Little Endian이다.

| CAN ID | Message | Signal generation |
|---|---|---|
| `0x386` | `WHL_SPD11` | 속도 10~130 km/h, 120초 주기 사인파 |
| `0x2B0` | `SAS11` | 18초/7초 파형 합성, 약 -5~+5 deg |
| `0x251` | `MDPS12` | 13초/5초 파형 합성, 약 -100~+100 |

송신 신호의 DBC 정의는 다음과 같다.

```text
WHL_SPD_FL          0|14@1+  factor 0.03125, offset 0
WHL_SPD_FR         16|14@1+  factor 0.03125, offset 0
SAS_Angle           0|16@1-  factor 0.1,     offset 0
CR_Mdps_StrColTq    0|11@1+  factor 1,       offset -1024
```

## Gateway → STM32 제어 수신

수신 필터는 아래 두 표준 ID만 FIFO0로 통과시킨다.

- `0x340 ADAS_STEER_CMD`: 목표 조향각, 토크 제한, Enable, Reason, Alive
- `0x341 ADAS_SPEED_CMD`: 목표 속도, 감속률, Enable, Reason, Alive

수신 명령이 100 ms 동안 들어오지 않으면 해당 `*_timed_out` 변수가 1로
변한다. 센서 파형 자체는 제어 명령에 따라 바뀌지 않는다. 이 단계의 목적은
양방향 변환과 판정 결과 전달을 검증하는 것이다.

## CubeIDE에서 실행

1. STM32CubeIDE에서 프로젝트를 Import한다.
2. Build Project를 실행한다.
3. NUCLEO-F446RE를 USB로 연결한다.
4. Run 또는 Debug를 눌러 보드에 다운로드한다.
5. Raspberry Pi Gateway 서비스를 실행한다.

게이트웨이에서 CAN 프레임을 확인한다.

```bash
candump -tz can0
```

정상 상태에서는 `386`, `2B0`, `251`이 반복해서 보인다. ADAS 명령이 들어오면
`340`, `341`도 보이고 Nucleo LD2가 수신 때마다 토글된다.

## CubeIDE Live Expressions

송신 데이터를 확인할 변수:

- `sim_wheel_speed_fl_kph`
- `sim_wheel_speed_fr_kph`
- `sim_steering_angle_deg`
- `sim_steering_column_torque`
- `can_tx_count`, `can_tx_error_count`

조향 명령 수신을 확인할 변수:

- `command_target_angle_deg`
- `command_torque_limit`
- `command_steering_enable`
- `command_steering_reason`
- `command_steering_alive`
- `command_steering_rx_count`
- `command_steering_timed_out`

속도 명령 수신을 확인할 변수:

- `command_target_speed_kph`
- `command_deceleration_percent`
- `command_speed_enable`
- `command_speed_reason`
- `command_speed_alive`
- `command_speed_rx_count`
- `command_speed_timed_out`

공통 진단 변수:

- `can_rx_count`
- `can_last_rx_id`
- `can_last_rx_dlc`
- `can_last_rx_data`
- `can_rx_error_count`
- `can_last_hal_error`

