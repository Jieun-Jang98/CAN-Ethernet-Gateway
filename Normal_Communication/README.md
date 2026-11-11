# 2026 KUSE CAN-Ethernet gateway — 정상 통신 구현

이 프로젝트는 CAN–Ethernet 게이트웨이의 정상 통신 동작을 구현하기 위한 프로젝트입니다. 공통 메시지 형식, CAN 데이터 인코딩·디코딩, 데이터 검증 및 토크 계산 로직을 포함합니다.

현재 1단계 구현 범위는 플랫폼 독립적인 공통 코드와 단위 테스트입니다. 실제 CAN·UDP 송수신 및 STM32·Gateway·ADAS 실행 프로그램은 이후 단계에서 구현할 예정입니다.

Stage 1 contains only platform-independent common code and unit tests. It does
not open CAN or UDP sockets and does not contain STM32 HAL, Gateway, or ADAS
executables.

## Build

```powershell
cmake -S . -B build
cmake --build build --config Release
ctest --test-dir build -C Release --output-on-failure
```

The common library and tests are implemented in C++17. Include `kuse/common.hpp` and use the `kuse` namespace. Public headers use `.hpp`; implementation files use `.cpp`.

