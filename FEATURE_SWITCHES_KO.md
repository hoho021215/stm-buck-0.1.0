# 0.1.0 기능 ON/OFF 값과 코드 위치

CubeIDE의 **Live Expressions**에서 `buck_feature_request`에 아래 표의 숫자를 씁니다. 펌웨어는 **다음 기동을 수락할 때** 이 값을 적용합니다. 같은 바이너리로 A/B를 비교하므로 소스를 편집하거나 다시 빌드할 필요가 없습니다. 아래 링크를 누르면 0.1.0 소스의 해당 줄이 열립니다.

## 기능 하나만 꺼서 비교하는 표

| 비교할 기능 | 모두 ON | 해당 기능만 OFF | 비트 정의 | 실제 제어식과 동작 |
|---|---:|---:|---|---|
| 입력전압 변화에 대한 피드포워드 | **7** | **6** | 값 1 · [Core/Inc/buck_app.h 12줄](Core/Inc/buck_app.h#L12) | ON에서는 기준전압을 실측 입력전압으로, OFF에서는 공칭 30 V로 나눠 기본 듀티를 계산합니다. [Core/Src/buck_app.c 231–232줄](Core/Src/buck_app.c#L231-L232) |
| 듀티 제한 시 적분기 보정(anti-windup) | **7** | **5** | 값 2 · [Core/Inc/buck_app.h 13줄](Core/Inc/buck_app.h#L13) | ON이면 적용 듀티와 요청 듀티 차이를 누적해 다음 전압 루프의 적분값을 보정합니다. OFF에서도 적분값은 −0.25~+0.25 범위로 제한합니다. [Core/Src/buck_app.c 239–240줄, 보정량 누적](Core/Src/buck_app.c#L239-L240) · [225–226줄, 적분 반영](Core/Src/buck_app.c#L225-L226) |
| 정상 운전 중 듀티 상승률 제한 | **7** | **3** | 값 4 · [Core/Inc/buck_app.h 14줄](Core/Inc/buck_app.h#L14) | ON이면 한 PWM 주기(20 µs)에 듀티가 최대 0.00002만 증가합니다. OFF에서도 기동 검사를 통과할 때까지는 같은 제한을 적용합니다. [Core/Src/buck_app.c 235–236줄, 상승 제한](Core/Src/buck_app.c#L235-L236) · [215–217줄, 기동 검사](Core/Src/buck_app.c#L215-L217) |

숫자는 **켜 둘 기능 값의 합**입니다. 예를 들어 `6 = 2 + 4`이므로 피드포워드만 꺼집니다. `0`은 세 비교 기능을 모두 끄는 조합입니다. 이 경우에도 소프트스타트, 전류에 따른 듀티 상한과 고장 차단은 작동합니다. 전체 조합의 기본값은 `1 + 2 + 4 = 7`입니다. [전체 비트 정의](Core/Inc/buck_app.h#L15) · [소프트스타트와 전류에 따른 듀티 상한](Core/Src/buck_app.c#L212-L222)

## CubeIDE에서 바꾸는 위치와 순서

아래는 배선·게이트 파형·보호 동작을 검증해 기동이 허용된 보드에서의 절차입니다. 배포본의 `BUCK_BOARD_VERIFIED` 기본값은 0이어서 전력단 기동이 차단됩니다. [Core/Inc/buck_board.h 11줄](Core/Inc/buck_board.h#L11)

1. **어디에 쓰나?** 정지 상태(`buck_debug.state == 0`)와 영점 보정 완료(`calibrated == 1`)를 확인합니다. Live Expressions에 **`buck_feature_request`**를 추가하고 **Value** 칸에 A 조건 `7`을 씁니다. **Core/Inc/buck_app.h 67줄**은 이 변수의 선언이며, 변수의 메모리는 **Core/Src/buck_app.c 32줄**에 정의돼 있습니다. `Buck_Init()`을 호출하면 요청값이 7로 초기화되므로 초기화 후에 씁니다. [선언 67줄](Core/Inc/buck_app.h#L67) · [정의 32줄](Core/Src/buck_app.c#L32) · [초기화 287–288줄](Core/Src/buck_app.c#L287-L288)
2. **언제 적용되나?** `buck_arm_request = 1`로 기동을 요청합니다. 기동 조건을 통과하면 요청값이 내부 제어용 `b.active_features`와 디버거 표시용 **`buck_debug.active_features`**에 복사됩니다. 운전 중 `buck_feature_request`에 다른 값을 써도 현재 제어는 이미 복사한 값을 사용합니다. [기동 검사와 적용 169–178줄](Core/Src/buck_app.c#L169-L178) · [표시 필드 선언 33줄](Core/Inc/buck_app.h#L33)
3. **어느 조합으로 측정했나?** `buck_debug.active_features == 7`과 `buck_debug.monitoring == 1`을 확인한 뒤 **`buck_measure_request = 1`**을 한 번 씁니다. `buck_measurement.status == 2`, `samples == 2500`이면 결과를 읽습니다. 결과의 `buck_measurement.active_features`에는 **그 50 ms 측정에 사용한 값**이 들어 있습니다. [측정 요청 변수 선언 27–29줄](Core/Inc/buck_measurement.h#L27-L29) · [조합 기록 31–35줄](Core/Src/buck_measurement.c#L31-L35) · [완료 처리 54–61줄](Core/Src/buck_measurement.c#L54-L61)
4. **B 조건은 어떻게 바꾸나?** `buck_arm_request = 2`로 정지합니다. 정지 처리와 명령 소비를 확인하고 출력이 0.5 V 이하로 내려오면, 표에서 고른 **6, 5, 3 중 하나**를 `buck_feature_request`에 씁니다. 같은 입력·부하 조건으로 2~3단계를 반복하세요. [정지 처리 141–143줄](Core/Src/buck_app.c#L141-L143) · [측정값과 비교 절차](README.md)
