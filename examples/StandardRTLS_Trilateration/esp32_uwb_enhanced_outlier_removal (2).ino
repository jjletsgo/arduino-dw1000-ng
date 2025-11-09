/*
 * ESP32 UWB Trilateration System - ENHANCED VERSION WITH OUTLIER REMOVAL
 * 추가/수정 기능:
 * 1. 칼만 필터 (위치 및 속도 추정)
 * 2. 이상치 완전 제거 (OUTLIER_REMOVAL 옵션)
 * 3. 복도 축 정렬 (Corridor Snapping)
 * 4. 앵커별 거리 오프셋 보정 (ANCHOR_OFFSETS)
 */

#include <micro_ros_arduino.h>
#include <stdio.h>
#include <rcl/rcl.h>
#include <rcl/error_handling.h>
#include <rclc/rclc.h>
#include <rclc/executor.h>

#include <std_msgs/msg/string.h>
#include <geometry_msgs/msg/point_stamped.h>

#include "BluetoothSerial.h"
#include <math.h>
#include "HardwareSerial.h"

// Bluetooth
BluetoothSerial SerialBT;
const char* btServerName = "ESP32_ANCHOR_MAIN";
boolean btConnected = false;

// LED
#define LED_PIN 2

// Error handling
#define RCCHECK(fn) { rcl_ret_t temp_rc = fn; if((temp_rc != RCL_RET_OK)){ros_connected = false;}}
#define RCSOFTCHECK(fn) { rcl_ret_t temp_rc = fn; if((temp_rc != RCL_RET_OK)){}}

#define UART_RX_PIN 16
#define UART_TX_PIN 17

// ROS connection status
boolean ros_connected = false;

// ROS2 objects
rcl_allocator_t allocator;
rclc_support_t support;
rcl_node_t node;
rclc_executor_t executor;

// Publishers
rcl_publisher_t pos_publisher;
rcl_publisher_t debug_publisher;

// Subscribers
rcl_subscription_t track_start_sub;
rcl_subscription_t track_stop_sub;

// Messages
geometry_msgs__msg__PointStamped pos_msg;
std_msgs__msg__String debug_msg;
std_msgs__msg__String track_start_msg;
std_msgs__msg__String track_stop_msg;

// Message buffers
char debug_buffer[256];
char track_buffer[32];
char frame_id_buf[32];

// Tracking state
boolean tracking_enabled = false;
int current_tag_id = 0;

// Anchor coordinates
const double ANCHOR_Z = 139.0;  // cm
const double TAG_Z = 11.5;      // cm (fixed)

const double HEIGHT_DIFF = fabs(ANCHOR_Z - TAG_Z);  // 127.5 cm
const double MIN_VALID_RANGE = HEIGHT_DIFF + 1.0;   // 최소 유효 거리

// Gauss-Newton parameters
const int MAX_ITERATIONS = 15;
const double CONVERGENCE_THRESHOLD = 0.01;  // cm

// ==================== 이상치 제거 설정 ====================
const bool ENABLE_OUTLIER_REMOVAL = true;    // true: 이상치 완전 제거, false: 감지만
const double OUTLIER_THRESHOLD = 2.0;        // 표준편차의 배수
const double MIN_OUTLIER_STDDEV = 10.0;      // 최소 표준편차 (cm) - 너무 작은 편차로 제거 방지

// ==================== 앵커별 거리 오프셋 설정 ====================
// 각 앵커의 체계적 오차를 보정하기 위한 오프셋 (cm 단위)
// 양수: 측정 거리에 더하기 (실제보다 짧게 측정되는 경우)
// 음수: 측정 거리에서 빼기 (실제보다 길게 측정되는 경우)
const double ANCHOR_OFFSETS[4] = {
    0.0,   // Anchor 1 오프셋 (예: 2.5 = 측정값에 2.5cm 추가)
    0.0,   // Anchor 2 오프셋 (예: -1.0 = 측정값에서 1cm 감소)  
    0.0,   // Anchor 3 오프셋
    0.0    // Anchor 4 오프셋
};

// NLOS detection thresholds
const float POWER_DIFF_THRESHOLD = 6.0;
const float QUALITY_THRESHOLD = 0.5;

const double DISTANCE_WEIGHT_FACTOR = 2.0;

// ========== 복도 축 정렬 구조체 및 데이터 ==========
enum SegType : uint8_t { SEG_VERT, SEG_HORIZ };

struct AxisSeg {
    SegType type;
    double fixed;  // 고정 좌표 (VERT면 x, HORIZ면 y)
    double r1;     // 범위 시작
    double r2;     // 범위 끝
};

// 복도 세그먼트 정의 (cm 단위)
const AxisSeg CORRIDORS[] = {
    {SEG_VERT,  20.0,  0.0,  180.0},   
    {SEG_VERT,  55.0,  147.5,  180.0},  
    {SEG_VERT,  85.0,  147.5,  180.0},   
    {SEG_VERT,  115.0,  147.5,  180.0},   
    {SEG_VERT,  145.0,  147.5,  180.0},  
    {SEG_VERT, 147.5,  92.5,  147.5},
    {SEG_VERT, 55.0,  60.0,  92.5},
    {SEG_VERT, 85.0,  60.0,  92.5},
    {SEG_VERT, 115.0,  60.0,  92.5},
    {SEG_VERT, 145.0,  60.0,  92.5},
    {SEG_HORIZ, 147.5,  20.0, 147.5},  
    {SEG_HORIZ, 140.0,  147.5, 180.0}, 
    {SEG_HORIZ, 92.5,  20.0, 147.5}, 
    {SEG_HORIZ, 100.0,  147.5, 180.0},   
};
const size_t N_CORRIDORS = sizeof(CORRIDORS) / sizeof(CORRIDORS[0]);

// ========== 칼만 필터 구조체 ==========
struct KalmanFilter2D {
    double x[4];      // 상태 벡터 [x, y, vx, vy]
    double P[4][4];   // 공분산 행렬
    double Q;         // 프로세스 노이즈
    double R;         // 측정 노이즈
    double dt;        // 시간 간격
    
    void init(double init_x, double init_y, double process_noise = 0.01, double measure_noise = 0.5) {
        x[0] = init_x; x[1] = init_y; x[2] = 0; x[3] = 0;
        Q = process_noise;
        R = measure_noise;
        dt = 0.1;  // 100ms
        
        for(int i = 0; i < 4; i++) {
            for(int j = 0; j < 4; j++) {
                P[i][j] = (i == j) ? 1000.0 : 0.0;
            }
        }
    }
    
    void predict() {
        double new_x[4];
        new_x[0] = x[0] + x[2] * dt;
        new_x[1] = x[1] + x[3] * dt;
        new_x[2] = x[2];
        new_x[3] = x[3];
        
        for(int i = 0; i < 4; i++) x[i] = new_x[i];
        
        double F[4][4] = {
            {1, 0, dt, 0},
            {0, 1, 0, dt},
            {0, 0, 1, 0},
            {0, 0, 0, 1}
        };
        
        double temp[4][4];
        for(int i = 0; i < 4; i++) {
            for(int j = 0; j < 4; j++) {
                temp[i][j] = 0;
                for(int k = 0; k < 4; k++) {
                    temp[i][j] += F[i][k] * P[k][j];
                }
            }
        }
        
        for(int i = 0; i < 4; i++) {
            for(int j = 0; j < 4; j++) {
                P[i][j] = 0;
                for(int k = 0; k < 4; k++) {
                    P[i][j] += temp[i][k] * F[j][k];
                }
                if(i == j) P[i][j] += Q;
            }
        }
    }
    
    void update(double meas_x, double meas_y) {
        double S[2][2];
        S[0][0] = P[0][0] + R;
        S[0][1] = P[0][1];
        S[1][0] = P[1][0];
        S[1][1] = P[1][1] + R;
        
        double det = S[0][0] * S[1][1] - S[0][1] * S[1][0];
        if(fabs(det) < 1e-6) return;
        
        double S_inv[2][2];
        S_inv[0][0] = S[1][1] / det;
        S_inv[0][1] = -S[0][1] / det;
        S_inv[1][0] = -S[1][0] / det;
        S_inv[1][1] = S[0][0] / det;
        
        double K[4][2];
        for(int i = 0; i < 4; i++) {
            K[i][0] = P[i][0] * S_inv[0][0] + P[i][1] * S_inv[1][0];
            K[i][1] = P[i][0] * S_inv[0][1] + P[i][1] * S_inv[1][1];
        }
        
        double y_err[2];
        y_err[0] = meas_x - x[0];
        y_err[1] = meas_y - x[1];
        
        for(int i = 0; i < 4; i++) {
            x[i] += K[i][0] * y_err[0] + K[i][1] * y_err[1];
        }
        
        double new_P[4][4];
        for(int i = 0; i < 4; i++) {
            for(int j = 0; j < 4; j++) {
                new_P[i][j] = P[i][j];
                if(i < 2 && j < 4) {
                    new_P[i][j] -= K[i][0] * P[0][j] + K[i][1] * P[1][j];
                }
            }
        }
        
        for(int i = 0; i < 4; i++) {
            for(int j = 0; j < 4; j++) {
                P[i][j] = new_P[i][j];
            }
        }
    }
    
    void getPosition(double& out_x, double& out_y) {
        out_x = x[0];
        out_y = x[1];
    }
};

// ========== 중간값 필터 ==========
class MedianFilter {
    double buffer[5];
    int index;
    int count;
    
public:
    MedianFilter() : index(0), count(0) {}
    
    double filter(double value) {
        buffer[index] = value;
        index = (index + 1) % 5;
        if(count < 5) count++;
        
        double sorted[5];
        for(int i = 0; i < count; i++) {
            sorted[i] = buffer[i];
        }
        
        for(int i = 0; i < count - 1; i++) {
            for(int j = 0; j < count - i - 1; j++) {
                if(sorted[j] > sorted[j + 1]) {
                    double temp = sorted[j];
                    sorted[j] = sorted[j + 1];
                    sorted[j + 1] = temp;
                }
            }
        }
        
        if(count > 0) {
            return sorted[count / 2];
        }
        return value;
    }
    
    void reset() {
        index = 0;
        count = 0;
    }
};

// Anchor coordinates
struct AnchorCoord {
    double x;
    double y;
    double z;
};

// 앵커 좌표 (음수 좌표 유지)
AnchorCoord anchors[4] = {
    {-30.0, -30.0, ANCHOR_Z},
    {-30.0, 230.0, ANCHOR_Z},
    {230.0, 230.0, ANCHOR_Z},
    {230.0, -30.0, ANCHOR_Z}
};

// UWB Data structure
struct UWBData {
    double range;
    float fpPower;
    float rxPower;
    float rxQuality;
    boolean valid;
    unsigned long timestamp;
};

UWBData uwbData[4];

// Position tracking
struct PositionHistory {
    double x;
    double y;
    bool valid;
    unsigned long timestamp;
};

PositionHistory lastPosition = {0, 0, false, 0};

// 칼만 필터와 중간값 필터 인스턴스
KalmanFilter2D kalmanFilter;
MedianFilter medianFilters[4];
bool kalmanInitialized = false;

// ========== 복도 축 정렬 함수 ==========
void snapToCorridors(double x, double y, double& sx, double& sy) {
    if (N_CORRIDORS == 0) { 
        sx = x; 
        sy = y; 
        return; 
    }

    double best_dist = 1e9;
    double best_x = x, best_y = y;
    int best_seg_idx = -1;

    for (size_t i = 0; i < N_CORRIDORS; i++) {
        const AxisSeg& seg = CORRIDORS[i];
        double cand_x, cand_y;
        
        if (seg.type == SEG_VERT) {
            cand_x = seg.fixed;
            cand_y = fmax(seg.r1, fmin(seg.r2, y));
        } else {
            cand_y = seg.fixed;
            cand_x = fmax(seg.r1, fmin(seg.r2, x));
        }
        
        double dx = x - cand_x;
        double dy = y - cand_y;
        double dist = sqrt(dx*dx + dy*dy);
        
        if (dist < best_dist) {
            best_dist = dist;
            best_x = cand_x;
            best_y = cand_y;
            best_seg_idx = i;
        }
    }
    
    // 항상 가장 가까운 복도로 스냅
    sx = best_x;
    sy = best_y;
    
    if (best_seg_idx >= 0) {
        const AxisSeg& seg = CORRIDORS[best_seg_idx];
        Serial2.print("Snapped to ");
        Serial2.print(seg.type == SEG_VERT ? "VERT" : "HORIZ");
        Serial2.print(" corridor #");
        Serial2.print(best_seg_idx);
        Serial2.print(" (dist: ");
        Serial2.print(best_dist, 1);
        Serial2.println("cm)");
    }
}

// 거리 검증 및 보정 함수 (오프셋 적용 추가)
double validateAndCorrectRange(double range_m, int anchorIdx) {
    double range_cm = range_m * 100.0;
    
    // 앵커별 오프셋 적용
    if (anchorIdx >= 0 && anchorIdx < 4) {
        range_cm += ANCHOR_OFFSETS[anchorIdx];
        if (ANCHOR_OFFSETS[anchorIdx] != 0) {
            Serial2.print("Anchor ");
            Serial2.print(anchorIdx + 1);
            Serial2.print(" offset applied: ");
            Serial2.print(ANCHOR_OFFSETS[anchorIdx], 1);
            Serial2.println("cm");
        }
    }
    
    if (range_cm < MIN_VALID_RANGE) {
        Serial2.print("Warning: Range too short (");
        Serial2.print(range_cm);
        Serial2.print("cm), corrected to minimum: ");
        Serial2.println(MIN_VALID_RANGE);
        range_cm = MIN_VALID_RANGE;
    }
    
    const double MAX_VALID_RANGE = 1000.0;
    if (range_cm > MAX_VALID_RANGE) {
        Serial2.print("Warning: Range too long (");
        Serial2.print(range_cm);
        Serial2.println("cm), measurement discarded");
        return -1.0;
    }
    
    return range_cm;
}

// 적응형 가중치 계산
double calculateAdaptiveWeight(const UWBData& data, double estimated_distance) {
    if (!data.valid) return 0.0;
    
    float powerDiff = fabs(data.rxPower - data.fpPower);
    double powerWeight = exp(-powerDiff / POWER_DIFF_THRESHOLD);
    double qualityWeight = data.rxQuality;
    
    double distanceWeight = 1.0;
    if (estimated_distance > 0) {
        distanceWeight = 1.0 / (1.0 + pow(estimated_distance / 100.0, DISTANCE_WEIGHT_FACTOR));
    }
    
    double freshnessWeight = 1.0;
    unsigned long age = millis() - data.timestamp;
    if (age > 1000) {
        freshnessWeight = 0.5;
    }
    
    double weight = powerWeight * qualityWeight * distanceWeight * freshnessWeight;
    
    if (weight < 0.01) weight = 0.01;
    
    return weight;
}

// 이상치 탐지 및 제거 함수
bool detectAndRemoveOutliers(double r2d[], int validIndices[], int& validCount) {
    if (validCount < 4 || !ENABLE_OUTLIER_REMOVAL) return false;
    
    // 평균 계산
    double mean = 0.0;
    for (int i = 0; i < validCount; i++) {
        mean += r2d[validIndices[i]];
    }
    mean /= validCount;
    
    // 표준편차 계산
    double variance = 0.0;
    for (int i = 0; i < validCount; i++) {
        double diff = r2d[validIndices[i]] - mean;
        variance += diff * diff;
    }
    variance /= validCount;
    double stddev = sqrt(variance);
    
    // 최소 표준편차 적용
    if (stddev < MIN_OUTLIER_STDDEV) {
        stddev = MIN_OUTLIER_STDDEV;
    }
    
    // 이상치 식별 및 제거
    bool outlierRemoved = false;
    int newValidCount = 0;
    int newValidIndices[4];
    
    for (int i = 0; i < validCount; i++) {
        int idx = validIndices[i];
        double deviation = fabs(r2d[idx] - mean) / stddev;
        
        if (deviation > OUTLIER_THRESHOLD) {
            Serial2.print("OUTLIER REMOVED: Anchor ");
            Serial2.print(idx + 1);
            Serial2.print(" (deviation: ");
            Serial2.print(deviation, 2);
            Serial2.print(", range: ");
            Serial2.print(r2d[idx], 1);
            Serial2.println("cm)");
            outlierRemoved = true;
            // 이 앵커는 제외
        } else {
            newValidIndices[newValidCount++] = idx;
        }
    }
    
    // 최소 3개의 앵커는 유지해야 함
    if (newValidCount >= 3) {
        validCount = newValidCount;
        for (int i = 0; i < validCount; i++) {
            validIndices[i] = newValidIndices[i];
        }
        
        if (outlierRemoved) {
            Serial2.print("Remaining anchors after outlier removal: ");
            for (int i = 0; i < validCount; i++) {
                Serial2.print(validIndices[i] + 1);
                if (i < validCount - 1) Serial2.print(", ");
            }
            Serial2.println();
        }
    } else {
        Serial2.println("Warning: Cannot remove outliers (would leave < 3 anchors)");
        outlierRemoved = false;
    }
    
    return outlierRemoved;
}

// 2D Weighted Least Squares with Gauss-Newton Method (이상치 제거 적용)
boolean calculatePosition2D_GaussNewton(double& x, double& y, int& iterations_used, bool& converged, int& anchorsUsed) {
    int validCount = 0;
    int validIndices[4];
    for (int i = 0; i < 4; i++) {
        if (uwbData[i].valid) {
            uwbData[i].range = medianFilters[i].filter(uwbData[i].range);
            validIndices[validCount++] = i;
        }
    }
    
    if (validCount < 3) {
        Serial2.println("Not enough valid measurements");
        return false;
    }
    
    double r2d[4];
    double weights[4];
    
    for (int i = 0; i < validCount; i++) {
        int idx = validIndices[i];
        double r3d = uwbData[idx].range;
        double r3d_sq = r3d * r3d;
        double height_sq = HEIGHT_DIFF * HEIGHT_DIFF;
        
        if (r3d_sq > height_sq) {
            r2d[idx] = sqrt(r3d_sq - height_sq);
        } else {
            r2d[idx] = 1.0;
            Serial2.print("Warning: Anchor ");
            Serial2.print(idx + 1);
            Serial2.println(" range less than height difference");
        }
    }
    
    // 이상치 탐지 및 제거
    bool outliersRemoved = detectAndRemoveOutliers(r2d, validIndices, validCount);
    anchorsUsed = validCount;
    
    // Initial guess
    if (lastPosition.valid && (millis() - lastPosition.timestamp < 1000)) {
        x = lastPosition.x;
        y = lastPosition.y;
    } else {
        x = 100.0;  // 태그 영역 중앙
        y = 100.0;
    }
    
    converged = false;
    
    // Gauss-Newton iterations
    for (int iter = 0; iter < MAX_ITERATIONS; iter++) {
        iterations_used = iter + 1;
        
        double J[4][2];
        double r[4];
        double JTW[2][4];
        double JTWJ[2][2];
        double JTWr[2];
        
        for (int i = 0; i < 2; i++) {
            for (int j = 0; j < 2; j++) {
                JTWJ[i][j] = 0.0;
            }
            JTWr[i] = 0.0;
        }
        
        for (int i = 0; i < validCount; i++) {
            int idx = validIndices[i];
            double dx = x - anchors[idx].x;
            double dy = y - anchors[idx].y;
            double estimated_dist = sqrt(dx * dx + dy * dy);
            
            weights[idx] = calculateAdaptiveWeight(uwbData[idx], estimated_dist);
        }
        
        for (int i = 0; i < validCount; i++) {
            int idx = validIndices[i];
            double dx = x - anchors[idx].x;
            double dy = y - anchors[idx].y;
            double dist = sqrt(dx * dx + dy * dy);
            
            if (dist < 0.001) dist = 0.001;
            
            J[i][0] = dx / dist;
            J[i][1] = dy / dist;
            
            r[i] = r2d[idx] - dist;
        }
        
        for (int i = 0; i < validCount; i++) {
            int idx = validIndices[i];
            double w = weights[idx];
            
            JTW[0][i] = J[i][0] * w;
            JTW[1][i] = J[i][1] * w;
            
            for (int j = 0; j < 2; j++) {
                for (int k = 0; k < 2; k++) {
                    JTWJ[j][k] += JTW[j][i] * J[i][k];
                }
                JTWr[j] += JTW[j][i] * r[i];
            }
        }
        
        double det = JTWJ[0][0] * JTWJ[1][1] - JTWJ[0][1] * JTWJ[1][0];
        
        if (fabs(det) < 0.0001) {
            Serial2.println("Warning: Singular matrix in Gauss-Newton");
            return false;
        }
        
        double delta_x = (JTWJ[1][1] * JTWr[0] - JTWJ[0][1] * JTWr[1]) / det;
        double delta_y = (JTWJ[0][0] * JTWr[1] - JTWJ[1][0] * JTWr[0]) / det;
        
        double step_size = sqrt(delta_x * delta_x + delta_y * delta_y);
        const double MAX_STEP = 50.0;
        if (step_size > MAX_STEP) {
            double scale = MAX_STEP / step_size;
            delta_x *= scale;
            delta_y *= scale;
        }
        
        x += delta_x;
        y += delta_y;
        
        double update_norm = sqrt(delta_x * delta_x + delta_y * delta_y);
        if (update_norm < CONVERGENCE_THRESHOLD) {
            converged = true;
            break;
        }
    }
    
    double raw_x = x, raw_y = y;
    
    // 태그 영역(0~200) 내로 제한
    const double TAG_MIN_X = 0.0, TAG_MAX_X = 200.0;
    const double TAG_MIN_Y = 0.0, TAG_MAX_Y = 200.0;
    
    bool out_of_bounds = (x < TAG_MIN_X || x > TAG_MAX_X || y < TAG_MIN_Y || y > TAG_MAX_Y);
    
    if (out_of_bounds) {
        Serial2.print("Position out of tag area (");
        Serial2.print(x, 1);
        Serial2.print(", ");
        Serial2.print(y, 1);
        Serial2.print(") -> clamped to ");
        
        x = fmax(TAG_MIN_X, fmin(TAG_MAX_X, x));
        y = fmax(TAG_MIN_Y, fmin(TAG_MAX_Y, y));
        
        Serial2.print("(");
        Serial2.print(x, 1);
        Serial2.print(", ");
        Serial2.print(y, 1);
        Serial2.println(")");
    }
    
    // 칼만 필터 적용
    if (!kalmanInitialized) {
        kalmanFilter.init(x, y);
        kalmanInitialized = true;
    } else {
        kalmanFilter.predict();
        kalmanFilter.update(x, y);
        kalmanFilter.getPosition(x, y);
    }
    
    // 복도 축 정렬 적용
    double snapped_x, snapped_y;
    snapToCorridors(x, y, snapped_x, snapped_y);
    x = snapped_x;
    y = snapped_y;
    
    Serial2.print("Final: raw(");
    Serial2.print(raw_x, 1);
    Serial2.print(",");
    Serial2.print(raw_y, 1);
    Serial2.print(") -> snapped(");
    Serial2.print(x, 1);
    Serial2.print(",");
    Serial2.print(y, 1);
    Serial2.print(") | Anchors used: ");
    Serial2.println(anchorsUsed);
    
    lastPosition.x = x;
    lastPosition.y = y;
    lastPosition.valid = true;
    lastPosition.timestamp = millis();
    
    return true;
}

// Track start callback
void track_start_callback(const void * msgin) {
    const std_msgs__msg__String * msg = (const std_msgs__msg__String *)msgin;
    
    char* str = (char*)msg->data.data;
    char* comma = strchr(str, ',');
    
    if (comma != NULL) {
        *comma = '\0';
        current_tag_id = atoi(comma + 1);
        tracking_enabled = true;
        
        lastPosition.valid = false;
        kalmanInitialized = false;
        for (int i = 0; i < 4; i++) {
            medianFilters[i].reset();
        }
        
        snprintf(debug_buffer, sizeof(debug_buffer), 
                 "[ROS_BRIDGE] Track start: tag_id=%d", current_tag_id);
        debug_msg.data.data = debug_buffer;
        debug_msg.data.size = strlen(debug_buffer);
        RCSOFTCHECK(rcl_publish(&debug_publisher, &debug_msg, NULL));
    }
}

// Track stop callback
void track_stop_callback(const void * msgin) {
    const std_msgs__msg__String * msg = (const std_msgs__msg__String *)msgin;
    
    char* str = (char*)msg->data.data;
    char* comma = strchr(str, ',');
    
    if (comma != NULL) {
        *comma = '\0';
        int tag_id = atoi(comma + 1);
        
        if (tag_id == current_tag_id) {
            tracking_enabled = false;
            
            snprintf(debug_buffer, sizeof(debug_buffer), 
                     "[ROS_BRIDGE] Track stop: tag_id=%d", tag_id);
            debug_msg.data.data = debug_buffer;
            debug_msg.data.size = strlen(debug_buffer);
            RCSOFTCHECK(rcl_publish(&debug_publisher, &debug_msg, NULL));
        }
    }
}

// Parse Bluetooth data with offset application
void parseBluetoothData() {
    static String btBuffer = "";
    static unsigned long lastDataReceived = 0;
    
    while (SerialBT.available()) {
        char c = SerialBT.read();
        
        if (c == '\n') {
            if (millis() - lastDataReceived > 1000) {
                Serial2.println("Receiving UWB data...");
                lastDataReceived = millis();
            }
            
            if (btBuffer.startsWith("Anchor_")) {
                int anchorNum = btBuffer.charAt(7) - '0';
                if (anchorNum >= 1 && anchorNum <= 4) {
                    int idx = anchorNum - 1;
                    
                    int rangeIdx = btBuffer.indexOf("Range:");
                    int fpIdx = btBuffer.indexOf("FP_Power:");
                    int rxIdx = btBuffer.indexOf("RX_Power:");
                    int qualityIdx = btBuffer.indexOf("RX_Quality:");
                    
                    if (rangeIdx != -1 && fpIdx != -1 && rxIdx != -1 && qualityIdx != -1) {
                        String rangeStr = btBuffer.substring(rangeIdx + 6, btBuffer.indexOf(',', rangeIdx));
                        String fpStr = btBuffer.substring(fpIdx + 9, btBuffer.indexOf(',', fpIdx));
                        String rxStr = btBuffer.substring(rxIdx + 9, btBuffer.indexOf(',', rxIdx));
                        String qualityStr = btBuffer.substring(qualityIdx + 11);
                        
                        // 오프셋이 적용된 거리 검증 및 보정
                        double range_cm = validateAndCorrectRange(rangeStr.toFloat(), idx);
                        
                        if (range_cm > 0) {
                            uwbData[idx].range = range_cm;
                            uwbData[idx].fpPower = fpStr.toFloat();
                            uwbData[idx].rxPower = rxStr.toFloat();
                            uwbData[idx].rxQuality = qualityStr.toFloat();
                            uwbData[idx].valid = true;
                            uwbData[idx].timestamp = millis();
                            
                            Serial2.print("A");
                            Serial2.print(anchorNum);
                            Serial2.print(": ");
                            Serial2.print(range_cm, 1);
                            Serial2.print("cm (Q:");
                            Serial2.print(uwbData[idx].rxQuality, 2);
                            if (ANCHOR_OFFSETS[idx] != 0) {
                                Serial2.print(",Offset:");
                                Serial2.print(ANCHOR_OFFSETS[idx], 1);
                            }
                            Serial2.println(")");
                        }
                    }
                }
            }
            else if (btBuffer.indexOf("========================") != -1) {
                processUWBData();
                
                for (int i = 0; i < 4; i++) {
                    uwbData[i].valid = false;
                }
            }
            
            btBuffer = "";
        } else {
            btBuffer += c;
        }
    }
}

// Process UWB data and publish position
void processUWBData() {
    double x, y;
    int iterations = 0;
    bool converged = false;
    int anchorsUsed = 0;
    
    if (calculatePosition2D_GaussNewton(x, y, iterations, converged, anchorsUsed)) {
        double x_m = x / 100.0;
        double y_m = y / 100.0;
        double z_m = TAG_Z / 100.0;
        
        snprintf(debug_buffer, sizeof(debug_buffer), 
                 "[POS] Tag_%d: (%.2f, %.2f, %.2f)m | Anchors:%d Iter:%d %s | OR:%s KF:✓ Snap:✓ | BT:%s ROS:%s", 
                 current_tag_id > 0 ? current_tag_id : 0,
                 x_m, y_m, z_m,
                 anchorsUsed,
                 iterations,
                 converged ? "✓" : "⚠",
                 ENABLE_OUTLIER_REMOVAL ? "ON" : "OFF",
                 btConnected ? "✓" : "✗",
                 ros_connected ? "✓" : "✗");
        
        Serial2.println(debug_buffer);
        
        if (ros_connected) {
            debug_msg.data.data = debug_buffer;
            debug_msg.data.size = strlen(debug_buffer);
            RCSOFTCHECK(rcl_publish(&debug_publisher, &debug_msg, NULL));
            
            if (tracking_enabled && current_tag_id > 0) {
                snprintf(frame_id_buf, sizeof(frame_id_buf), "tag_%d", current_tag_id);
                
                pos_msg.header.frame_id.data = frame_id_buf;
                pos_msg.header.frame_id.size = strlen(frame_id_buf);
                pos_msg.header.stamp.sec = 0;
                pos_msg.header.stamp.nanosec = 0;
                
                pos_msg.point.x = x_m;
                pos_msg.point.y = y_m;
                pos_msg.point.z = z_m;
                
                RCSOFTCHECK(rcl_publish(&pos_publisher, &pos_msg, NULL));
            }
        }
    } else {
        snprintf(debug_buffer, sizeof(debug_buffer), 
                 "[ERROR] Position calculation failed | BT:%s ROS:%s", 
                 btConnected ? "✓" : "✗",
                 ros_connected ? "✓" : "✗");
        
        Serial2.println(debug_buffer);
        
        if (ros_connected) {
            debug_msg.data.data = debug_buffer;
            debug_msg.data.size = strlen(debug_buffer);
            RCSOFTCHECK(rcl_publish(&debug_publisher, &debug_msg, NULL));
        }
    }
}

void setup() {
    Serial.begin(115200);
    Serial2.begin(115200, SERIAL_8N1, UART_RX_PIN, UART_TX_PIN);
    Serial2.println("\n=== ESP32 UWB Trilateration (Enhanced with Outlier Removal) ===");
    Serial2.println("Features:");
    Serial2.println("- Kalman Filter (2D position + velocity)");
    Serial2.println("- Median Filter (per anchor)");
    Serial2.println("- Outlier REMOVAL (completely removes outliers)");
    Serial2.println("- Anchor-specific distance offsets");
    Serial2.println("- Corridor Axis Snapping");
    Serial2.println("- Tag area: (0,0) to (200,200)");
    
    // 오프셋 설정 표시
    Serial2.println("\nAnchor Offsets (cm):");
    for (int i = 0; i < 4; i++) {
        Serial2.print("  Anchor ");
        Serial2.print(i + 1);
        Serial2.print(": ");
        Serial2.println(ANCHOR_OFFSETS[i], 1);
    }
    
    Serial2.println("==========================================\n");
    
    SerialBT.begin("ESP32_UWB_CLIENT", true);
    Serial2.println("Connecting to anchor...");
    
    btConnected = SerialBT.connect(btServerName);
    
    if(btConnected) {
        Serial2.println("✓ Bluetooth connected");
    } else {
        Serial2.println("✗ Bluetooth connection failed, retrying...");
        int retryCount = 0;
        while(!btConnected && retryCount < 5) {
            delay(2000);
            btConnected = SerialBT.connect(btServerName);
            retryCount++;
            Serial2.print("Retry ");
            Serial2.print(retryCount);
            Serial2.println("/5");
        }
    }
    
    for (int i = 0; i < 4; i++) {
        uwbData[i].valid = false;
        uwbData[i].timestamp = 0;
    }
    
    lastPosition.valid = false;
    kalmanInitialized = false;
    
    Serial2.println("Connecting to MicroROS...");
    
    set_microros_transports();
    delay(2000);
    
    allocator = rcl_get_default_allocator();
    ros_connected = true;
    
    RCCHECK(rclc_support_init(&support, 0, NULL, &allocator));
    
    if (ros_connected) {
        RCCHECK(rclc_node_init_default(&node, "uwb_trilateration_node", "", &support));
        
        if (ros_connected) {
            RCCHECK(rclc_publisher_init_default(
                &pos_publisher,
                &node,
                ROSIDL_GET_MSG_TYPE_SUPPORT(geometry_msgs, msg, PointStamped),
                "/uwb/pos"));
            
            RCCHECK(rclc_publisher_init_default(
                &debug_publisher,
                &node,
                ROSIDL_GET_MSG_TYPE_SUPPORT(std_msgs, msg, String),
                "/uwb/debug"));
            
            RCCHECK(rclc_subscription_init_default(
                &track_start_sub,
                &node,
                ROSIDL_GET_MSG_TYPE_SUPPORT(std_msgs, msg, String),
                "/uwb/track_start"));
            
            RCCHECK(rclc_subscription_init_default(
                &track_stop_sub,
                &node,
                ROSIDL_GET_MSG_TYPE_SUPPORT(std_msgs, msg, String),
                "/uwb/track_stop"));
            
            if (ros_connected) {
                RCCHECK(rclc_executor_init(&executor, &support.context, 2, &allocator));
                RCCHECK(rclc_executor_add_subscription(&executor, &track_start_sub, &track_start_msg, 
                                                       &track_start_callback, ON_NEW_DATA));
                RCCHECK(rclc_executor_add_subscription(&executor, &track_stop_sub, &track_stop_msg, 
                                                       &track_stop_callback, ON_NEW_DATA));
            }
        }
    }
    
    if (ros_connected) {
        debug_msg.data.data = debug_buffer;
        debug_msg.data.capacity = sizeof(debug_buffer);
        debug_msg.data.size = 0;
        
        track_start_msg.data.data = track_buffer;
        track_start_msg.data.capacity = sizeof(track_buffer);
        track_start_msg.data.size = 0;
        
        track_stop_msg.data.data = track_buffer;
        track_stop_msg.data.capacity = sizeof(track_buffer);
        track_stop_msg.data.size = 0;
        
        pos_msg.header.frame_id.data = (char*)malloc(32);
        pos_msg.header.frame_id.capacity = 32;
        
        Serial2.println("✓ MicroROS connected");
        
        snprintf(debug_buffer, sizeof(debug_buffer), 
                 "[INIT] UWB Trilateration Ready (KF + OR + Offset + Snap)");
        debug_msg.data.data = debug_buffer;
        debug_msg.data.size = strlen(debug_buffer);
        RCSOFTCHECK(rcl_publish(&debug_publisher, &debug_msg, NULL));
    } else {
        Serial2.println("✗ MicroROS failed - Standalone mode");
    }
    
    Serial2.println("\n=== System Ready ===\n");
}

void loop() {
    if (!SerialBT.connected()) {
        if (btConnected) {
            btConnected = false;
            Serial2.println("⚠ Bluetooth disconnected!");
        }
        
        static unsigned long lastReconnectAttempt = 0;
        if (millis() - lastReconnectAttempt > 5000) {
            lastReconnectAttempt = millis();
            btConnected = SerialBT.connect(btServerName);
            if (btConnected) {
                Serial2.println("✓ Bluetooth reconnected");
            }
        }
    } else {
        btConnected = true;
        parseBluetoothData();
    }
    
    if (ros_connected) {
        RCSOFTCHECK(rclc_executor_spin_some(&executor, RCL_MS_TO_NS(10)));
    }
    
    delay(5);
}
