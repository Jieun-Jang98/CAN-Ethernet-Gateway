# Raspberry Pi CAN–Ethernet Gateway (C++17)

Raspberry Pi Gateway에서 SocketCAN `can0`와 UDP Ethernet 사이를 변환하는
프로그램이다.

## Fixed configuration

- Gateway Ethernet: `192.168.50.11/24`
- ADAS Ethernet: `192.168.50.12/24`
- CAN: Classical CAN, Standard 11-bit, 500 kbit/s
- CAN hardware: MCP2515-based RS485 CAN HAT in **CAN** mode
- State UDP: Gateway -> ADAS, port 5000, 20 ms
- Command UDP: ADAS -> Gateway, port 5001
- ADAS command timeout: 100 ms

상세 바이트 배치는 [PROTOCOL.md](PROTOCOL.md)에 정리되어 있다. CANoe,
SavvyCAN 또는 cantools에서 확인할 수 있는 `gateway_custom.dbc`도 포함한다.

## 1. Install tools on the Gateway Raspberry Pi

```bash
sudo apt update
sudo apt install -y build-essential cmake can-utils unzip
```

MCP2515 장치 트리 설정을 완료하고 재부팅한 뒤 다음 명령에서 `can0`가 보여야
한다.

```bash
ip link show can0
```

## 2. Build

프로젝트 폴더 안에서 실행한다.

```bash
mkdir -p build
cmake -S . -B build -DCMAKE_BUILD_TYPE=Release
cmake --build build -j2
ctest --test-dir build --output-on-failure
```

## 3. Configure CAN and Ethernet

```bash
chmod +x setup_can.sh
./setup_can.sh
sudo ip addr replace 192.168.50.11/24 dev eth0
sudo ip link set eth0 up
ip -4 -br addr show eth0
```

상대 ADAS Raspberry Pi의 `eth0`는 `192.168.50.12/24`로 설정한다.

```bash
ping -c 3 192.168.50.12
```

## 4. Run

```bash
./build/can_gateway
```

기본값을 바꿀 때만 옵션을 준다.

```bash
./build/can_gateway \
  --can can0 \
  --local-ip 192.168.50.11 \
  --adas-ip 192.168.50.12 \
  --state-port 5000 \
  --command-port 5001
```

종료는 `Ctrl+C`이다. `sudo` 없이 CAN 소켓 접근이 거부되면 우선 아래처럼
실행해 확인할 수 있다.

```bash
sudo ./build/can_gateway
```

## 5. Observe CAN during testing

별도 SSH 창에서 센서 입력과 제어 출력을 확인한다.

```bash
candump -tz can0
```

CAN 버스 오류와 누적 통계는 다음 명령으로 확인한다.

```bash
ip -details -statistics link show can0
```

## 6. Install as an automatic service

게이트웨이를 SSH 세션과 무관하게 실행하고 부팅 때 자동 시작하려면 다음 명령을
한 번 실행한다. 설치 과정에서 빌드와 단위 테스트도 함께 수행된다.

```bash
chmod +x install_service.sh
./install_service.sh
```

상태 및 실시간 로그 확인 명령은 다음과 같다.

```bash
sudo systemctl status can-gateway.service
sudo journalctl -u can-gateway.service -f
```

서비스를 중지하거나 다시 시작할 때는 다음 명령을 사용한다.

```bash
sudo systemctl stop can-gateway.service
sudo systemctl restart can-gateway.service
```

## Hardware notes

- HAT 점퍼를 RS485가 아닌 **CAN** 위치에 둔다.
- CAN_H끼리, CAN_L끼리 연결한다.
- HAT이 CAN 버스 끝이라면 120-ohm 종단 스위치를 ON으로 둔다.
- CAN 장치들은 공통 GND를 연결한다.
- RS485 A/B 단자는 이 프로젝트에서 사용하지 않는다.

