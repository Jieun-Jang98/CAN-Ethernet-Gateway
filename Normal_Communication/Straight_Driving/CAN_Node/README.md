# 직진 보조 시나리오 STM32 CAN 노드

대상: NUCLEO-F446RE, SN65HVD230, Classical CAN 500 kbps. 이 노드는 차량 상태만 모사하며 실제 조향 모터를 구동하지 않습니다.

## 동작

- 20 ms마다 WHL_SPD11 (`0x386`, 8바이트), SAS11 (`0x2B0`, 5바이트), MDPS12 (`0x251`, 8바이트)를 송신합니다. 네 바퀴 속도는 모두 50 km/h이고 운전자 토크는 0입니다.
- 최초 유효한 활성 LKAS11 (`0x340`, 8바이트)을 받으면 12초 시나리오 시간을 시작합니다. 0~2초: 0°, 2~4초: +2°, 4~6초: +1°, 6~8초: 0°, 8~10초: -2°, 10~12초: 0°이며, 시나리오는 계속 반복됩니다. 해제 명령을 받아도 입력 시나리오는 진행됩니다.
- LKAS11의 Standard/Data/DLC, CRC8, 4비트 카운터를 검사합니다. 잘못된 프레임은 마지막 유효 수신 시각과 로컬 출력을 갱신하지 않습니다. 100 ms 타임아웃 또는 CAN bus-off 때 로컬 출력 토크를 0으로 해제합니다.
- 유효한 LKAS11에서 **Torque_Request와 Enable 두 값을 각각 수신**해 `kuse_rx_torque_request`, `kuse_rx_enable`에 저장합니다. Gateway의 안전 제한 결과가 CAN에 담기므로, 이 토크는 원래 UDP 요청과 다를 수 있습니다. 실제 로컬 적용값은 `kuse_output_torque`, `kuse_output_enable`입니다.
- 타임아웃 후 카운터 기준을 지우고, 순서가 이어지는 유효한 토크 0 LKAS11 세 개로 회복합니다. 세 번째 프레임까지 0으로 출력하고 다음 유효 프레임부터 명령을 적용합니다. Enable 값은 회복 조건에 쓰지 않습니다.
- 각 송신 카운터는 `HAL_CAN_AddTxMessage` 성공 뒤에만 증가합니다. 전송 큐에 넣지 못한 프레임은 다음 주기에 같은 카운터로 재시도합니다.

## 프로젝트 구성

`board/`는 STM32CubeIDE에서 바로 가져올 수 있는 F446RE 프로젝트입니다. `board/Core/Src/main.c`의 USER CODE 영역에서 C++ `app_main_run`을 호출합니다. `board/Core/Src/app_main.cpp`가 HAL CAN 송수신을 담당하고 `straight_node.cpp`가 시나리오와 수신 검증을 담당합니다. CubeIDE 빌드에는 `board/Core/Inc/kuse/`와 `board/Core/Src/`에 복사한 공통 코덱을 포함합니다. 상위 `../../common/`을 수정할 때는 이 복사본도 동기화해야 합니다.

보드 연결은 PB8=CAN1_RX, PB9=CAN1_TX, CANH/CANL, 공통 GND와 버스 양 끝 120 Ω 종단입니다. SN65HVD230 전원과 입출력 레벨은 모듈 사양을 따르세요. `board/KUSE_Straight_CAN_Log.ioc`는 84 MHz 시스템 클록과 42 MHz APB1에서 CAN prescaler 6, BS1 11 TQ, BS2 2 TQ로 500 kbps를 설정합니다.

## STM32CubeIDE에서 실행

1. NUCLEO-F446RE를 ST-LINK USB로 연결합니다.
2. STM32CubeIDE의 **File > Open Projects from File System...**에서 이 폴더의 `board/`를 선택해 `KUSE_Straight_CAN_Log` 프로젝트를 가져옵니다.
3. Project Explorer에서 프로젝트를 선택하고 **Project > Build Project**를 실행합니다. Console에 오류가 없고 `Debug/KUSE_Straight_CAN_Log.elf`가 생성되는지 확인합니다.
4. **Run > Run As > STM32 C/C++ Application**으로 보드에 기록하고 실행합니다. 디버거 Watch 변수를 보려면 **Debug As > STM32 C/C++ Application**을 선택하고 Resume을 누릅니다.

## CubeIDE에서 CAN 송수신 로그 보기

NUCLEO-F446RE의 ST-LINK USB 가상 COM 포트로 USART2(PA2/PA3) 로그를 보냅니다. 추가 USB-CAN 어댑터 없이 **이 STM32가 송신 큐에 넣은 프레임과 수신한 LKAS11**을 볼 수 있습니다.

1. Windows 장치 관리자 **포트(COM 및 LPT)**에서 `STMicroelectronics STLink Virtual COM Port (COM숫자)`를 확인합니다.
2. CubeIDE **Window > Show View > Other... > Terminal > Terminal**을 엽니다. **Open a Terminal**에서 **Local Terminal**을 선택합니다. 설치된 CubeIDE에 Serial Terminal 항목이 없어도 이 방법으로 IDE 안에서 로그를 볼 수 있습니다.
3. Terminal에서 `cd /d C:\path\to\board`로 이 프로젝트의 `board` 폴더로 이동한 뒤 `powershell -NoProfile -ExecutionPolicy Bypass -File .\view_can_log.ps1 -Port COM3`을 실행합니다. `COM3`는 실제 장치 관리자에 표시된 포트로 바꿉니다. 연결된 COM 포트가 하나뿐이면 `-Port COM3`을 생략할 수 있습니다. 스크립트는 **230400 baud, 8N1**로 연결합니다.
4. 보드의 검은 RESET 버튼을 누르거나 CubeIDE에서 Run/Debug 후 Resume을 누릅니다. **Console**의 ST-LINK 다운로드 메시지가 아니라 **Terminal** 뷰에서 아래 로그를 확인합니다. 종료는 `Ctrl+C`입니다.

```text
BOOT KUSE_Straight_CAN_Log CAN=500000 UART=230400 8N1
123 TXQ 386 8 40 06 40 06 40 46 40 46
123 TXQ 2B0 5 00 00 00 00 00
123 TXQ 251 8 00 04 00 00 00 00 00 00
143 RX 340 8 00 00 FD 0B 00 00 17 00 req=-3 en=1 out=-3/1 OK
```

첫 숫자는 부팅 후 경과 밀리초입니다. `TXQ`는 HAL CAN 송신 큐 등록 성공이며 실제 상대 수신 증거는 아닙니다. `RX`의 `req`와 `en`은 **STM32가 LKAS11에서 받은 두 값**, `out`은 안전/회복 상태를 반영한 로컬 적용값입니다. 잘못된 프레임은 `BAD_CRC`, `BAD_COUNTER`, `BAD_COMMAND`, `BAD_FRAME`으로 표시됩니다. 실제 버스 전송 여부는 Gateway의 `candump can0` 또는 별도 CAN 분석기로 교차 확인합니다.

## CMake 대체 빌드

STM32CubeIDE의 ARM GNU 도구가 `PATH`에 있는 환경에서:

```powershell
cmake -S Straight_Driving/CAN_Node -B Straight_Driving/CAN_Node/build -G Ninja
cmake --build Straight_Driving/CAN_Node/build
```

결과는 `Straight_Driving/CAN_Node/build/KUSE_Straight_CAN_Log.elf`와 `.hex`입니다. `board/KUSE_Straight_CAN_Log.ioc`를 CubeIDE에서 열어 하드웨어 설정을 검토할 수 있습니다. CubeMX에서 다시 생성할 때 `main.c`의 USER CODE 호출과 C++ 소스의 빌드 포함 여부를 확인하세요.

호스트 단위 시험은 상위 `Normal_Communication` 디렉터리에서 `cmake -S . -B build-host`, `cmake --build build-host --config Release`, `ctest --test-dir build-host -C Release --output-on-failure` 순서로 실행합니다.

CubeIDE 디버거의 `kuse_tx_ok`, `kuse_rx_ok`, `kuse_rx_torque_request`, `kuse_rx_enable`, `kuse_rx_bad_crc`, `kuse_rx_bad_counter`, `kuse_command_timed_out`, `kuse_command_recovering`, `kuse_output_torque`, `kuse_output_enable`, `kuse_uart_log_failed`, `kuse_scenario_cycle`로 송수신 상태를 확인할 수 있습니다.
