# 4개 앵커 RTLS 시스템 설정 가이드

## 개요
이 시스템은 4개의 앵커(A, B, C, D)와 1개의 태그를 사용하여 TWR(Two-Way Ranging) 기반 실시간 위치 추적을 수행합니다.
태그는 각 앵커와의 거리 측정 후 다음 정보를 블루투스로 전송합니다:
- 거리값 (Range)
- First Path Power (FP Power)
- Receive Power (RX Power)
- Receive Quality (RX Quality)

## 시스템 구성

### 앵커 구성
1. **Anchor A (Main)** - Device Address: 0x01
   - 마스터 앵커
   - 태그의 BLINK를 받아 레인징 시작
   - 다른 앵커들의 측정 결과를 수집

2. **Anchor B** - Device Address: 0x02
   - 슬레이브 앵커
   - 측정 후 Anchor C로 전달

3. **Anchor C** - Device Address: 0x03
   - 슬레이브 앵커
   - 측정 후 Anchor D로 전달

4. **Anchor D** - Device Address: 0x04
   - 슬레이브 앵커 (마지막)
   - 측정 후 사이클 종료

### 태그
- **Tag** - Device Address: 0x05
  - 주기적으로 BLINK 전송
  - 4개 앵커와 순차적으로 거리 측정
  - 모든 측정 완료 후 블루투스로 데이터 전송
  - 블루투스 장치명: "ESP32_UWB_TAG"

## 레인징 순서
```
Tag → Anchor A (Main) → Anchor B → Anchor C → Anchor D → 완료
```

## 업로드 순서

### 1단계: 모든 앵커 업로드
```
1. Anchor A (Main): examples/StandardRTLSAnchorMain_TWR/
2. Anchor B: examples/StandardRTLSAnchorB_TWR/
3. Anchor C: examples/StandardRTLSAnchorC_TWR/
4. Anchor D: examples/StandardRTLSAnchorD_TWR/
```

### 2단계: 태그 업로드
```
Tag: examples/StandardRTLSTag_TWR/
```

⚠️ **중요**: ESP32에 업로드할 때 BluetoothSerial 라이브러리가 필요합니다.

## 앵커 위치 설정

`StandardRTLSAnchorMain_TWR.ino`에서 앵커 위치를 설정할 수 있습니다:

```cpp
Position position_self = {0,0};      // Anchor A (Main) 위치
Position position_B = {3,0};         // Anchor B 위치
Position position_C = {3,2.5};       // Anchor C 위치
Position position_D = {0,2.5};       // Anchor D 위치
```

단위: 미터(m)

## 블루투스 데이터 포맷

태그에서 전송하는 블루투스 데이터 형식:

```
=== UWB Ranging Data ===
Anchor_1,Range:2.345,FP_Power:-85.23,RX_Power:-82.45,RX_Quality:12.34
Anchor_2,Range:3.456,FP_Power:-87.12,RX_Power:-84.23,RX_Quality:11.23
Anchor_3,Range:1.234,FP_Power:-83.45,RX_Power:-80.12,RX_Quality:13.45
Anchor_4,Range:2.789,FP_Power:-86.78,RX_Power:-83.56,RX_Quality:12.67
========================
```

### 데이터 파싱 예제 (삼변측량용 ESP32)

```cpp
#include "BluetoothSerial.h"

BluetoothSerial SerialBT;

void setup() {
    Serial.begin(115200);
    SerialBT.begin("ESP32_TRILATERATION");
    
    // Connect to tag: "ESP32_UWB_TAG"
}

void loop() {
    if (SerialBT.available()) {
        String line = SerialBT.readStringUntil('\n');
        
        if (line.startsWith("Anchor_")) {
            // Parse data
            int anchorNum = line.substring(7, line.indexOf(',')).toInt();
            
            int rangeIdx = line.indexOf("Range:") + 6;
            int fpIdx = line.indexOf("FP_Power:");
            float range = line.substring(rangeIdx, fpIdx - 1).toFloat();
            
            int rxIdx = line.indexOf("RX_Power:");
            float fpPower = line.substring(fpIdx + 9, rxIdx - 1).toFloat();
            
            int qualIdx = line.indexOf("RX_Quality:");
            float rxPower = line.substring(rxIdx + 9, qualIdx - 1).toFloat();
            
            float rxQuality = line.substring(qualIdx + 11).toFloat();
            
            Serial.printf("Anchor %d: Range=%.3f, FP=%.2f, RX=%.2f, Q=%.2f\n",
                         anchorNum, range, fpPower, rxPower, rxQuality);
            
            // 여기서 삼변측량 계산 수행
        }
    }
}
```

## 시리얼 모니터 출력

### Anchor A (Main)
```
Range: 2.345 m    RX power: -82.45 dBm
Range from: 2 = 3.456
Range from: 3 = 1.234
Range from: 4 = 2.789
Found position - x: 1.23 y: 1.45
=== All 4 Anchor Ranges ===
Anchor A (Main): 2.345 m
Anchor B: 3.456 m
Anchor C: 1.234 m
Anchor D: 2.789 m
===========================
```

### Anchor B, C, D
```
Range: 3.456 m    RX power: -84.23 dBm
```

### Tag
```
--- Starting new ranging cycle ---
Range request successful
Ranging with anchor: 0x1
Anchor 1 - Range: 2.345m, RX: -82.45dBm
Ranging with anchor: 0x2
Anchor 2 - Range: 3.456m, RX: -84.23dBm
Ranging with anchor: 0x3
Anchor 3 - Range: 1.234m, RX: -80.12dBm
Ranging with anchor: 0x4
Anchor 4 - Range: 2.789m, RX: -83.56dBm
Completed ranging with 4 anchors
Ranging cycle completed successfully
=== Data sent via Bluetooth ===
Anchor 1: Range=2.345m, FP=-85.23dBm, RX=-82.45dBm, Q=12.34
Anchor 2: Range=3.456m, FP=-87.12dBm, RX=-84.23dBm, Q=11.23
Anchor 3: Range=1.234m, FP=-83.45dBm, RX=-80.12dBm, Q=13.45
Anchor 4: Range=2.789m, FP=-86.78dBm, RX=-83.56dBm, Q=12.67
```

## 설정 조정

### 블링크 레이트 (Blink Rate)
태그의 측정 주기를 조정하려면 `StandardRTLSTag_TWR.ino`에서:
```cpp
volatile uint32_t blink_rate = 200;  // 밀리초 단위 (200ms = 5Hz)
```

### 안테나 딜레이 (Antenna Delay)
모든 디바이스에서 동일하게 설정되어야 합니다:
```cpp
DW1000Ng::setAntennaDelay(16436);
```

### 타임아웃 설정
각 앵커와 태그의 타임아웃 설정:
```cpp
DW1000Ng::setPreambleDetectionTimeout(64);
DW1000Ng::setSfdDetectionTimeout(273);
DW1000Ng::setReceiveFrameWaitTimeoutPeriod(5000);  // 앵커
DW1000Ng::setReceiveFrameWaitTimeoutPeriod(2000);  // 태그
```

## 트러블슈팅

### 문제: 태그가 앵커를 찾지 못함
- 모든 디바이스가 같은 채널(CHANNEL_5) 사용 확인
- 안테나 연결 확인
- 앵커들이 먼저 켜져있는지 확인

### 문제: 블루투스 연결 안됨
- ESP32 사용 확인 (ESP8266은 BluetoothSerial 미지원)
- 블루투스 장치명 "ESP32_UWB_TAG" 검색
- 시리얼 모니터에서 "Bluetooth initialized" 메시지 확인

### 문제: 일부 앵커와만 통신됨
- 각 앵커의 시리얼 출력 확인
- next_anchor 설정 확인:
  - Anchor B: next_anchor = 4
  - Anchor C: next_anchor = 4
  - Anchor Main: next_anchor = 2
- 앵커 간 거리가 너무 멀지 않은지 확인 (권장: 10m 이내)

### 문제: 거리 측정이 부정확함
- 안테나 딜레이 캘리브레이션 수행
- 앵커 위치 좌표 정확히 설정
- 주변 장애물 확인
- LOS(Line of Sight) 확보

## 추가 개선 사항

### 1. 거리 계산 정확도 향상
현재 Tag 코드의 거리 계산은 단순화되어 있습니다. 더 정확한 거리 측정을 위해서는 `DW1000NgRanging::computeRangeAsymmetric()` 함수를 사용하여 모든 타임스탬프를 고려한 계산이 필요합니다.

### 2. 4앵커 삼변측량
3개 앵커의 삼변측량은 Main 앵커에 구현되어 있으나, 4개 앵커를 사용한 보다 정확한 위치 추정을 위해서는 최소제곱법(Least Squares) 알고리즘 구현이 필요합니다.

### 3. 데이터 검증
블루투스로 전송되는 데이터에 체크섬이나 시퀀스 번호를 추가하여 데이터 무결성을 보장할 수 있습니다.

## 참고 자료
- DW1000 User Manual
- IEEE 802.15.4-2011 Standard
- ISO/IEC 24730-62:2013 (RTLS TWR)

