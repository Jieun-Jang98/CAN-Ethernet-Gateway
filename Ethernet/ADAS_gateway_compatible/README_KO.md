# 업로드한 Gateway 규격에 맞춘 ADAS

이 버전은 저장소의 Gateway UDP/CAN 규격과 STM32 송수신 코드를 한 세트로 맞춘 버전입니다. 이전 `ADAS_speed_control.zip`의 0x01/0x02/0x03 규격을 대체하므로 **현재 저장소의 Gateway, ADAS, STM32 파일을 함께 사용**하세요.

## 실행

ADAS Raspberry Pi Ethernet 주소는 Gateway 설정에 맞춰 `192.168.50.12`, Gateway는 `192.168.50.11`입니다. 프로그램이 OS의 IP를 변경하지는 않습니다. ADAS는 UDP 5000에서 상태를 받고 `192.168.50.11:5001`로 두 종류의 명령을 보냅니다.

압축을 풀고 같은 폴더에서:

```sh
g++ -std=c++17 -O2 -Wall -Wextra -Wpedantic adas_controller.cpp -o adas_controller
./adas_controller
```

기본 목표는 60km/h입니다. 실행 인자로 바꿀 수 있습니다.

```sh
./adas_controller 80
# 다른 Gateway 주소 사용 시
./adas_controller 60 192.168.50.11
```

목표 허용 범위는 벤치용 0~120km/h입니다. 정지 목표는 정지 유지/브레이크 홀드 기능을 보장하지 않습니다. 종료는 Ctrl+C이며 정상 종료 시 두 비활성 명령을 한 번 보냅니다. 강제 종료/네트워크 단절 시 이 패킷의 전송은 보장되지 않습니다.

## 이번 제어의 의미

**조향:** Gateway의 입력은 토크 요청이 아니라 `TargetAngle`, `TorqueLimit`입니다. 따라서 기존 `-0.5 × angle` 조향 토크를 목표 각도로 그대로 넣지 않았습니다. 직진 목표를 0도로 두고 활성화 시 측정 조향각부터 0도 방향으로 최대10도/s씩 목표를 이동합니다. 예를 들어 현재 +4도이면 정상20ms 첫 요청은 +3.8도, 이후 +3.6도 순서입니다. 패킷의 각도 해상도는 0.1도입니다. 이것은 차선 인식/차선 중앙 추종이 아니라 조향각0도를 향하는 벤치 시나리오입니다.

**속도:** Gateway가 받는 `TargetSpeed`에 설정 목표 속도를 보냅니다. 현재 속도가 목표보다 0.5km/h 이상 높을 때 추가 감속 비율을 계산합니다.

```
speed = (FL + FR) / 2
감속 요청[%] = clamp(5 × (speed - target - 0.5), 0, 30)
```

감속 내부 값은 최대20 percentage points/s로 변합니다. 정상20ms에서는0.4씩 변하고 정수%로 반올림해 전송합니다. 과속이 해소되면 감속은 바로0으로 해제합니다. 이 규칙은 **벤치용 초기값**이며 percent와 실제 감속도 사이의 보정은 없습니다. 이전 PI 가속도 출력을 임의의 감속%로 변환하지 않았습니다.

목표보다 느리면 목표속도60/감속0을 보내지만, **이 규격에는 별도의 가속 페달이나 양의 가속도 필드가 없습니다.** 목표속도를 받아 가속/속도 유지를 수행할 하위 제어기 또는 가상 차량 모델이 있어야 합니다. Gateway는 목표를 CAN으로 전달할 뿐 자체 속도 유지 제어기는 아닙니다. 따라서 이 ADAS만 변경해서 실제 속도가 자동으로60에 수렴하는 것은 아닙니다.

## 기존 허용 조건 유지

| 평균 속도 | 허용 측정 각도 | 운전자 토크 개입 기준 | 출력 TorqueLimit |
|---|---|---|---|
| 30km/h 미만 | ±30도 | 절댓값10 초과 | 10 |
| 30~80km/h 미만 | ±10도 | 절댓값5 초과 | 5 |
| 80km/h 이상 | ±3도 | 절댓값2 초과 | 2 |

FL/FR는 각각0~250km/h, 좌우 차이는15km/h 이하만 허용합니다. 원래 운전자 토크 기준의 단위는 DBC에서 명시되지 않았습니다. 출력 TorqueLimit도 Gateway의0~200 범위 안에서 기존 모의 한계10/5/2를 보수적으로 유지했으며 실제 액추에이터의 Nm로 검증한 값은 아닙니다.

측정 조향각이 현재 속도 구간의 허용 범위를 넘으면 조향 제어를 끄지 않고 `enable=1`, `TargetAngle=0도`, `reason=4`를 보내 즉시 중앙 복귀를 요구합니다. 잘못된 상태, 좌우 휠 속도 이상 또는 운전자 토크 개입 시에는 **조향·속도를 모두 비활성화**하고 목표각도/토크제한/목표속도/감속값을0으로 보냅니다. 재활성화 시 정상 목표 각도는 다시 측정 각도에서 시작합니다.

## Gateway와 맞춘 UDP

모든 multi-byte 값은 **little-endian**, sequence는 **uint16**입니다.

| 메시지 | type | 전체 길이 | 배치 |
|---|---|---|---|
| 상태 수신 | 0x10 | 12 | type[0], seq[1:2], payload length8[3], FL[4:5], FR[6:7], angle[8:9], torque[10:11] |
| 조향 송신 | 0x20 | 10 | type[0], seq[1:2], payload length6[3], target angle[4:5], torque limit[6:7], enable[8], reason[9] |
| 속도 송신 | 0x21 | 10 | type[0], seq[1:2], payload length6[3], target speed[4:5], decel%[6], enable[7], reason[8], reserved0[9] |

속도 단위0.01km/h, 각도 단위0.1도, torque limit은uint16, 측정 운전자 토크는int16입니다. 두 명령은 같은 sequence로 약20ms마다 별도 UDP 데이터그램으로 송신합니다. Gateway는 두 타입을 독립 전달하며 같은 주기의 두 명령이 함께 전달되는 것은 보장되지 않습니다.

Reason 정의는 Gateway 허용범위0~15 내에서 이 ADAS가 지정합니다:
0 정상, 1 상태 timeout, 2 잘못된 값/속도, 3 좌우 속도 차이, 4 각도 초과, 5 운전자 개입,
6 잘못된 목표, 7 잘못된dt, 8 상태 미수신, 9 정상 종료.
Gateway는 reason의 의미를 판단하지 않고 CAN에 전달합니다.

## 통신 끊김과 순서 검사

- 지정 Gateway IP에서 온 상태만 수락합니다. Gateway 상태 송신 소켓은 임시 source port를 쓰므로 source port=5001을 요구하지 않습니다.
- 이전 상태보다 modulo65536으로1~32767 증가한 sequence만 수락합니다. 중복/역순은 freshness를 갱신하지 않습니다. 65535→0은 정상입니다.
- 상태가100ms 이상 갱신되지 않으면 두 비활성 명령을 약20ms마다 송신합니다. 잘못된 물리 상태는 수신 시각을 갱신하지 않으며 다음 제어 주기에서 해제합니다.
- timeout이 지난 뒤에도 동일 sequence 재전송으로 활성화하지 않습니다. Gateway가 sequence를0으로 초기화해 과거 값처럼 보이면 ADAS도 재시작하여 동기화해야 할 수 있습니다.
- Gateway가 각 CAN 센서를200ms까지 fresh로 간주해 새 sequence로 상태를 계속 보냅니다. 따라서 ADAS의100ms timeout은 **UDP 상태 갱신 기준**이고, CAN 센서 정지 시점부터 반드시100ms 안에 해제된다는 의미는 아닙니다. 대략200ms+100ms 및 스케줄 지연이 걸릴 수 있습니다.
- 업로드한 Gateway의 명령100ms timeout은 로그만 출력하고 비활성 CAN 프레임을 생성하지 않습니다. ADAS 프로세스가 죽으면 STM32/하위 수신기의 watchdog이 필요합니다. 이 ADAS 변경으로 해당 Gateway 동작을 보완한 것은 아닙니다.

## STM32 시나리오와 연결할 때

STM32는 휠 속도와 조향각을 자연스러운 사인파로 만들고 20ms마다 CAN으로 송신합니다. 조향 칼럼 토크는 DBC 정수 변환 후 항상 -2~+2 범위에 머물도록 작은 파형으로 제한했으므로 고속 구간에서도 정상 시험 중 운전자 개입으로 오판하지 않습니다. ADAS는 허용 각도를 벗어나면 활성 상태로 목표각도0도 복귀 명령을 보내며, 과속이면 설정 목표 속도와 감속 명령도 함께 보냅니다.

현재 STM32 송신기는 명령을 수신해 센서 파형 자체를 바꾸는 폐루프 차량 모델은 아닙니다. 따라서 CAN0x340/0x341 명령이 수신되는지는 확인할 수 있지만, 명령 때문에 다음 센서 값이 실제로0도나 목표 속도로 수렴하는 시험은 별도의 가상 차량 모델을 추가해야 합니다. 여기의0x340은 프로젝트 자체 정의이며 실제 현대차LKAS11 payload와 같지 않습니다.

## 파일과 검증

- `adas_controller.cpp`, `control.hpp`: ADAS 실행 코드.
- `gateway_reference/protocol.hpp`, `protocol.cpp`: 업로드한 Gateway 원본을 테스트용으로 그대로 복사. 실제 Gateway를 교체하는 파일이 아닙니다.
- `test_compatibility.cpp`: 양방향 UDP 및 Gateway의 CAN 인코딩까지 검증.
- `adas_original.cpp.txt`: 최초 업로드 ADAS 백업.
- `CMakeLists.txt`: CMake 빌드/테스트 설정.

```sh
g++ -std=c++17 -O2 -Wall -Wextra -Wpedantic test_compatibility.cpp gateway_reference/protocol.cpp -o test_compatibility
./test_compatibility
# 또는
cmake -S . -B build -DCMAKE_BUILD_TYPE=Release
cmake --build build
ctest --test-dir build --output-on-failure
```

Windows MSVC14.51 C++17 `/W4`로 컨트롤러 전체와 테스트를 컴파일했고 경고 없이 성공했습니다.
네이티브 테스트 실행도 통과했습니다: Gateway가 만든 상태의 실제 ADAS 디코딩,
ADAS가 만든 명령의 실제 Gateway 디코딩, CAN 목표각도/속도/감속/enable/counter 매핑,
양음수·단위·고정 바이트 기대값·16bit wrap·중복/역순·timeout·오류 해제 및 각도0도 복귀 시나리오.
테스트는 `assert` 대신 실행 중 검사를 사용하므로 Release/NDEBUG에서도 검사합니다.

이 호환성 테스트는 패킷 인코딩/디코딩과 제어 판단을 검증합니다. 실제 Raspberry Pi UDP/CAN 배선 및 장시간 운용은 별도의 통합 시험이 필요합니다. 이전 버전의 가속도PI/가상 차량 수렴 테스트 결과를 이 버전의 결과로 간주하면 안 됩니다. 기존 `ADAS_speed_control`의 시뮬레이터는 이전 UDP 규격이므로 이 버전과 호환되지 않습니다.

