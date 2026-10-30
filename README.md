# CAN–Ethernet Bidirectional Security Gateway

### CAN–Ethernet 양방향 보안 Gateway Prototype 구현 및 검증

> 실제 차량의 CAN 데이터와 OpenDBC를 기반으로 CAN–Ethernet 간 양방향 데이터 변환 Gateway를 구현하고, Rule-based 보안 필터를 적용하여 비정상 메시지의 탐지 및 차단 성능을 검증하는 프로젝트

**Team:** KUSE | 김주희 · 장지은  
**Project Type:** Automotive Network / Embedded System / Vehicle Cybersecurity  
**Status:** In Development

---

## 1. Project Overview

최근 차량의 전자·전기 아키텍처(E/E Architecture)는 다양한 ECU가 CAN 및 Ethernet을 통해 연결되는 구조로 발전하고 있다.

이러한 환경에서 Gateway는 서로 다른 네트워크 간 데이터 변환과 전달을 수행하는 핵심 구성 요소이며, 네트워크 간 연결이 확대됨에 따라 비정상 메시지와 공격 데이터가 다른 네트워크로 전파될 가능성도 고려해야 한다.

본 프로젝트는 **Hyundai Sonata 2020–2023의 실제 CAN 데이터와 OpenDBC**를 기반으로 차량 상태 및 조향 제어 신호를 CAN과 Ethernet 간 양방향으로 변환·전달하는 Gateway Prototype을 구현한다.

정상적인 통신 및 Signal Translation 기능을 검증한 후, 공격 시나리오를 구성하고 Rule-based 보안 정책을 적용하여 비정상 메시지의 네트워크 간 전파를 차단하는 것을 목표로 한다.

최종적으로 보안 기능 적용 전·후의 공격 탐지 성능, 오탐률 및 Gateway 처리 지연을 정량적으로 비교·분석한다.

### Project Objectives

- CAN ↔ Ethernet 양방향 통신 Gateway 구현
- OpenDBC 기반 CAN Signal Decoding / Encoding 및 데이터 Mapping
- Vehicle State ↔ Steering Command 기반 ADAS 통합 시나리오 구성
- Rule-based 비정상 메시지 탐지 및 차단 기능 구현
- 공격 주입을 통한 보안 정책 검증
- 보안 적용 전·후 통신 및 보안 성능 정량 평가

---

## 2. System Architecture

### 2.1 시스템 구성

본 프로젝트는 CAN Network, Gateway, Ethernet Network의 세 영역으로 구성된다.

```text
                  CAN Network
                       |
          +-------------------------+
          |                         |
     Normal ECU                Attack ECU
     (STM32)                    (STM32)
          |                         |
          +------------+------------+
                       |
                  Classical CAN
                       |
              +------------------+
              |     Gateway      |
              |  Raspberry Pi 4B |
              |                  |
              |  CAN Interface   |
              |  SocketCAN       |
              |  DBC Translation |
              |  Security Filter |
              |  UDP/IP          |
              +------------------+
                       |
                  Ethernet
                       |
              +------------------+
              |    ADAS Node     |
              |  Ethernet Host   |
              |                  |
              |  Vehicle State   |
              |  Steering Logic  |
              |  Command Output  |
              +------------------+
```

### 2.2 Hardware Configuration

| Component | Hardware | Role |
|---|---|---|
| Normal ECU | STM32 NUCLEO-F446RE + CAN Transceiver | 정상 CAN 메시지 송신 및 Steering Command 수신 |
| Attack ECU | STM32 NUCLEO-F446RE + CAN Transceiver | 비정상 CAN Frame 생성 및 공격 주입 |
| Gateway | Raspberry Pi 4B + CAN Interface | 양방향 데이터 변환, 보안 검사 및 메시지 전달 |
| Ethernet Node | Laptop → Raspberry Pi 4B | ADAS Controller 모사 및 Steering Command 생성 |
| Monitoring | Laptop | CAN/UDP 패킷 분석 및 실험 로그 수집 |

초기 개발 단계에서는 Laptop을 Ethernet Host로 활용하여 Gateway의 양방향 통신 기능을 검증한다.

이후 Raspberry Pi 기반 Ethernet Node로 전환하여 독립적인 ADAS 모사 환경을 구성할 예정이다.

---

## 3. Vehicle Data & Communication

### 3.1 Reference Vehicle

본 프로젝트에서는 실제 차량 통신 규격을 참고하기 위해 다음 데이터를 활용한다.

| Category | Reference |
|---|---|
| Reference Vehicle | Hyundai Sonata 2020–2023 |
| CAN Database | OpenDBC Hyundai Classical CAN DBC |
| Vehicle Data | commaCarSegments |
| CAN Protocol | Classical CAN |
| Ethernet Protocol | IPv4 / UDP |

실제 차량의 CAN Message ID, DLC, Signal Start Bit, Length, Scale 및 Endian 정보는 OpenDBC 정의를 기준으로 사용한다.

단, Ethernet Application Payload는 Gateway Prototype의 기능 및 보안 검증 목적에 맞게 별도로 설계한다.

### 3.2 CAN Signal Definition

**CAN → Ethernet: Vehicle State**

| CAN Signal | Description |
|---|---|
| WHL_SPD_FL | Front Left Wheel Speed |
| WHL_SPD_FR | Front Right Wheel Speed |
| SAS_Angle | Steering Angle |
| CR_Mdps_StrColTq | Steering Column Torque |

**Ethernet → CAN: Steering Command**

| CAN Signal | Description |
|---|---|
| CR_Lkas_StrToqReq | Steering Torque Request |
| CF_Lkas_ActToi | Steering Control Activation |

Gateway는 DBC에 정의된 Signal 정보를 기반으로 CAN Raw Data와 물리값 사이의 변환을 수행한다.

```text
Decoding:
Physical Value = Raw Value × Scale + Offset

Encoding:
Raw Value = (Physical Value - Offset) / Scale
```

---

## 4. Gateway Implementation

### 4.1 CAN → Ethernet

CAN Network에서 수신한 차량 상태 데이터를 Ethernet 기반 ADAS Node로 전달한다.

**Processing Flow**

```text
CAN Frame Reception
        ↓
CAN ID / DLC Validation
        ↓
DBC-based Signal Decoding
        ↓
Security Validation
        ↓
Physical Value Conversion
        ↓
Signal Buffer Update
        ↓
Ethernet Payload Mapping
        ↓
Binary Encoding / Packing
        ↓
UDP Transmission
        ↓
Ethernet Node
```

Gateway는 CAN Frame에서 필요한 Signal을 추출하고, 사전에 정의된 Mapping 규칙에 따라 Ethernet Payload로 변환한다.

Vehicle State 데이터는 다음 두 종류의 Payload로 구성한다.

- **Speed State:** 좌·우 전륜 속도
- **Steering State:** 조향각 및 Steering Column Torque

### 4.2 Ethernet → CAN

Ethernet 기반 ADAS Node에서 생성한 Steering Command를 CAN Network로 전달한다.

**Processing Flow**

```text
Steering Command Generation
        ↓
Ethernet Payload Encoding
        ↓
UDP Transmission
        ↓
Gateway UDP Reception
        ↓
Payload Validation
        ↓
Binary Unpacking
        ↓
Security Validation
        ↓
CAN Signal Mapping
        ↓
DBC-based CAN Encoding
        ↓
CAN Frame Generation
        ↓
CAN Transmission
        ↓
CAN Node
```

Gateway는 Ethernet Payload를 수신하여 유효성을 검사한 후, OpenDBC 기반 Signal Mapping 및 Encoding을 수행한다.

이 과정에서 대상 CAN Message의 전체 데이터 필드를 구성하고, 필요한 Counter 및 Checksum을 처리하여 CAN Frame을 송신한다.

---

## 5. ADAS Integration Scenario

본 프로젝트는 차량 상태 정보와 조향 제어 명령을 연계한 ADAS 통신 시나리오를 구성한다.

### Scenario

**Step 1. Vehicle State Transmission**

Normal ECU가 좌·우 Wheel Speed, Steering Angle 및 Steering Column Torque를 CAN Frame으로 송신한다.

**Step 2. Gateway Translation**

Gateway는 CAN Signal을 Decoding하여 Ethernet Payload로 변환한 후 ADAS Node로 전달한다.

**Step 3. Steering Command Generation**

ADAS Node는 수신한 Vehicle State와 사전에 정의된 Steering Target을 기반으로 조향 제어 활성 여부를 판단하고 Steering Torque Command
