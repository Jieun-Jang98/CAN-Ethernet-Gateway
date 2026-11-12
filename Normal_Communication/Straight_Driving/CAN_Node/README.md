# 직진 보조 시나리오 STM32 CAN 노드

대상: NUCLEO-F446RE, SN65HVD230, Classical CAN 500 kbps. 이 노드는 차량 상태만 모사하며 실제 조향 모터를 구동하지 않습니다.

## 동작

- 20 ms마다 WHL_SPD11 (`0x386`, 8바이트), SAS11 (`0x2B0`, 5바이트), MDPS12 (`0x251`, 8바이트)를 송신합니다. 네 바퀴 속도는 모두 50 km/h이고 운전자 토크는 0입니다.
- 최초 유효한 활성 LKAS11 (`0x340`, 8바이트)을 받으면 12초 시나리오 시간을 시작합니다. 0~2초: 0°, 2~4초: +2°, 4~6초: +1°, 6~8초: 0°, 8~10초: -2°, 10~12초: 0°이며, 시나리오는 계속 반복됩니다. 해제 명령을 받아도 입력 시나리오는 진행됩니다.
- LKAS11의 Standard/Data/DLC, CRC8, 4비트 카운터를 검사합니다. 잘못된 프레임은 마지막 유효 수신 시각과 로컬 출력을 갱신하지 않습니다. 100 ms 타임아웃 또는 CAN bus-off 때 로컬 출력 토크를 0으로 해제합니다.
- 타임아웃 후 카운터 기준을 지우고, 순서가 이어지는 유효한 토크 0 LKAS11 세 개로 회복합니다. 세 번째 프레임까지 0으로 출력하고 다음 유효 프레임부터 명령을 적용합니다. Enable 값은 회복 조건에 쓰지 않습니다.
- 각 송신 카운터는 `HAL_CAN_AddTxMessage` 성공 뒤에만 증가합니다. 전송 큐에 넣지 못한 프레임은 다음 주기에 같은 카운터로 재시도합니다.

## 프로젝트 구성

`board/`는 기존 F446RE CubeMX 생성 프로젝트를 복사한 하드웨어 설정입니다. `board/Core/Src/main.c`의 USER CODE 영역에서 C++ `app_main_run`을 호출합니다. `app_main.cpp`가 HAL CAN 송수신을 담당하고 `straight_node.cpp`가 시나리오와 수신 검증을 담당합니다. 공통 CAN 신호 코덱과 CRC는 `../../common/`에서 직접 컴파일합니다.

보드 연결은 PB8=CAN1_RX, PB9=CAN1_TX, CANH/CANL, 공통 GND와 버스 양 끝 120 Ω 종단입니다. SN65HVD230 전원과 입출력 레벨은 모듈 사양을 따르세요. `board/KUSE_Straight_CAN.ioc`는 84 MHz 시스템 클록과 42 MHz APB1에서 CAN prescaler 6, BS1 11 TQ, BS2 2 TQ로 500 kbps를 설정합니다.

## 빌드

STM32CubeIDE의 ARM GNU 도구가 `PATH`에 있는 환경에서:

```powershell
cmake -S Straight_Driving/CAN_Node -B Straight_Driving/CAN_Node/build -G Ninja
cmake --build Straight_Driving/CAN_Node/build
```

결과는 `Straight_Driving/CAN_Node/build/KUSE_Straight_CAN.elf`와 `.hex`입니다. `board/KUSE_Straight_CAN.ioc`를 CubeIDE에서 열어 하드웨어 설정을 검토할 수 있습니다. CubeMX에서 다시 생성할 때 `main.c`의 USER CODE 호출과 C++ 소스의 빌드 포함 여부를 확인하세요.

호스트 단위 시험은 상위 `Normal_Communication` 디렉터리에서 `cmake -S . -B build-host`, `cmake --build build-host --config Release`, `ctest --test-dir build-host -C Release --output-on-failure` 순서로 실행합니다.

CubeIDE 디버거의 `kuse_tx_ok`, `kuse_rx_ok`, `kuse_rx_bad_crc`, `kuse_rx_bad_counter`, `kuse_command_timed_out`, `kuse_command_recovering`, `kuse_output_torque`, `kuse_scenario_cycle`로 송수신 상태를 확인할 수 있습니다.
