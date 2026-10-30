# STM32 → Raspberry Pi CAN 송신 초기 설정

프로젝트: C:\STM32_Projects\CAN_TX_Test
보드: NUCLEO-F446RE (STM32F446RE)

- CAN1 Normal 모드 (외부 통신)
- PB8 = CAN1_RX, PB9 = CAN1_TX
- SYSCLK 84 MHz, APB1 42 MHz
- 500 kbps: Prescaler 6, BS1 11 TQ, BS2 2 TQ, SJW 1 TQ
- 표준 ID 0x100, DLC 4, 데이터 01 02 03 04
- 주기 100 ms, 자동 재전송 활성화
- 빌드: Debug / 오류 0개, 경고 0개 확인

아직 보드에 업로드하지 않았고 실제 CAN 수신도 검증하지 않았습니다.

## 보드 연결 후

1. STM32를 ST-LINK USB로 노트북에 연결합니다.
2. PB9 → 트랜시버 TX 입력, PB8 ← 트랜시버 RX 출력으로 연결합니다.
3. 트랜시버 전원/GND, CAN H/L, 공통 GND, 종단저항을 확인합니다.
4. 라즈베리파이의 can0를 500 kbps 수신 상태로 준비합니다.
5. CubeIDE에서 CAN_TX_Test를 선택하여 Debug → Resume(F8)를 실행합니다.
6. 라즈베리파이 candump can0에서 다음 프레임을 확인합니다.

    can0  100  [4]  01 02 03 04

## 디버거 변수

- tx_count: 하드웨어가 수락한 송신 요청 수
- tx_complete_count: TXOK로 확인한 전송 완료 수
- tx_failed_count: 요청 실패 또는 전송 실패 수
- tx_pending: 전송 완료 대기 여부
- tx_free: 비어 있는 송신 메일박스 수
- can_esr: CAN 하드웨어 오류 상태 레지스터
- can_error: HAL 오류 코드 (단독으로 전송 성공 여부를 판단하지 않음)

수신 노드가 없으면 ACK를 받지 못해 송신 요청이 대기할 수 있습니다.
현재 코드는 요청을 하나씩 처리하므로, 이런 경우 tx_count가 1에서 멈출 수 있습니다.
