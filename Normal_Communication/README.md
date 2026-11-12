# 2026 KUSE CAN-Ethernet gateway — 정상 통신 구현

이 프로젝트는 CAN–Ethernet 게이트웨이의 정상 통신 동작을 구현하기 위한 프로젝트입니다. 공통 메시지 형식, CAN 데이터 인코딩·디코딩, 데이터 검증 및 토크 계산 로직을 포함합니다.

공통 코드와 단위 테스트 외에 직진 보조 시나리오용 STM32 CAN 노드가 `Straight_Driving/CAN_Node/`에 있습니다. Gateway·ADAS 실행 프로그램은 아직 포함하지 않습니다.

The STM32 node sends Classical CAN status frames and receives LKAS11 through
HAL CAN1. See `Straight_Driving/CAN_Node/README.md` for its build and board setup.

## Build

```powershell
cmake -S . -B build
cmake --build build --config Release
ctest --test-dir build -C Release --output-on-failure
```

The common library and tests are implemented in C++17. Include `kuse/common.hpp` and use the `kuse` namespace. Public headers use `.hpp`; implementation files use `.cpp`.

