/*
   This example code is in the Public Domain (or CC0 licensed, at your option.)

   Unless required by applicable law or agreed to in writing, this
   software is distributed on an "AS IS" BASIS, WITHOUT WARRANTIES OR
   CONDITIONS OF ANY KIND, either express or implied.
*/

/* 
 * StandardRTLSAnchorMain_TWR.ino
 * 
 * This is an example master anchor in a RTLS using two way ranging ISO/IEC 24730-62_2013 messages
 * Modified to support 4 anchors and Bluetooth data transmission
 */

 #include <DW1000Ng.hpp>
 #include <DW1000NgUtils.hpp>
 #include <DW1000NgRanging.hpp>
 #include <DW1000NgRTLS.hpp>
 
 // Bluetooth for ESP32
 #if defined(ESP32)
 #include "BluetoothSerial.h"
 BluetoothSerial SerialBT;
 #endif
 
// connection pins
const uint8_t PIN_SCK = 18;
const uint8_t PIN_MOSI = 23;
const uint8_t PIN_MISO = 19;
const uint8_t PIN_SS = 4;
const uint8_t PIN_RST = 15; 
const uint8_t PIN_IRQ = 17;

// Extended Unique Identifier register. 64-bit device identifier. Register file: 0x01
const char EUI[] = "AA:BB:CC:DD:EE:FF:00:01";

// Maximum frame size for DW1000
#define MAX_FRAME_SIZE 127
 
 // Structure to store ranging data for each anchor
 typedef struct AnchorRangingData {
     double range;
     float fpPower;
     float rxPower;
     float rxQuality;
     boolean valid;
 } AnchorRangingData;
 
 double range_self;
 double range_B;
 double range_C;
 double range_D;
 
 // Store detailed data for each anchor
 AnchorRangingData anchorData[4]; // 0=A(self), 1=B, 2=C, 3=D
 
boolean received_B = false;
boolean received_C = false;

// Timeout for anchor ranging sequence
unsigned long last_blink_time = 0;
const unsigned long ANCHOR_TIMEOUT = 2000; // 2 seconds

byte target_eui[8];
byte tag_shortAddress[] = {0x05, 0x00};
 
 byte anchor_b[] = {0x02, 0x00};
 byte anchor_c[] = {0x03, 0x00};
 byte anchor_d[] = {0x04, 0x00};

 uint16_t next_anchor = 2;
 
 device_configuration_t DEFAULT_CONFIG = {
     false,
     true,
     true,
     true,
     false,
     SFDMode::STANDARD_SFD,
     Channel::CHANNEL_5,
     DataRate::RATE_850KBPS,
     PulseFrequency::FREQ_64MHZ,
     PreambleLength::LEN_256,
     PreambleCode::CODE_9
 };
 
 frame_filtering_configuration_t ANCHOR_FRAME_FILTER_CONFIG = {
     false,
     false,
     true,
     false,
     false,
     false,
     false,
     true /* This allows blink frames */
 };
 
 void setup() {
     // DEBUG monitoring
     Serial.begin(115200);
     Serial.println(F("### DW1000Ng-arduino-ranging-anchorMain (4 Anchors + BT) ###"));
     
     // Initialize Bluetooth
     #if defined(ESP32)
     SerialBT.begin("ESP32_ANCHOR_MAIN"); // Bluetooth device name
     Serial.println("Bluetooth initialized: ESP32_ANCHOR_MAIN");
     #endif
     
     // Initialize anchor data
     for(int i = 0; i < 4; i++) {
         anchorData[i].valid = false;
     }
     
     // initialize the driver
     DW1000Ng::initializeNoInterrupt(PIN_SS, PIN_RST);
     Serial.println(F("DW1000Ng initialized ..."));
     // general configuration
     DW1000Ng::applyConfiguration(DEFAULT_CONFIG);
     DW1000Ng::enableFrameFiltering(ANCHOR_FRAME_FILTER_CONFIG);
     
     DW1000Ng::setEUI(EUI);
 
     DW1000Ng::setPreambleDetectionTimeout(64);
     DW1000Ng::setSfdDetectionTimeout(273);
     DW1000Ng::setReceiveFrameWaitTimeoutPeriod(5000);
 
     DW1000Ng::setNetworkId(RTLS_APP_ID);
     DW1000Ng::setDeviceAddress(1);
     
     DW1000Ng::setAntennaDelay(16436);
     
     Serial.println(F("Committed configuration ..."));
     // DEBUG chip info and registers pretty printed
     char msg[128];
     DW1000Ng::getPrintableDeviceIdentifier(msg);
     Serial.print("Device ID: "); Serial.println(msg);
     DW1000Ng::getPrintableExtendedUniqueIdentifier(msg);
     Serial.print("Unique ID: "); Serial.println(msg);
     DW1000Ng::getPrintableNetworkIdAndShortAddress(msg);
     Serial.print("Network ID & Device Address: "); Serial.println(msg);
     DW1000Ng::getPrintableDeviceMode(msg);
     Serial.print("Device mode: "); Serial.println(msg);    
 }
 
// Function to send ranging data via Bluetooth (buffered for performance)
void sendBluetoothData() {
    #if defined(ESP32)
    // Larger buffer with safety margin to prevent overflow
    char btBuffer[400];
    int len = 0;
    const int MAX_LEN = sizeof(btBuffer) - 1;
    
    len += snprintf(btBuffer + len, MAX_LEN - len, 
                    "=== UWB Ranging Data ===\n");
    
    // Send data for each anchor with buffer overflow protection
    if(anchorData[0].valid && len < MAX_LEN - 100) {
        len += snprintf(btBuffer + len, MAX_LEN - len,
                       "Anchor_1,Range:%.3f,FP_Power:%.2f,RX_Power:%.2f,RX_Quality:%.2f\n",
                       anchorData[0].range, anchorData[0].fpPower, 
                       anchorData[0].rxPower, anchorData[0].rxQuality);
    }
    
    if(anchorData[1].valid && len < MAX_LEN - 100) {
        len += snprintf(btBuffer + len, MAX_LEN - len,
                       "Anchor_2,Range:%.3f,FP_Power:%.2f,RX_Power:%.2f,RX_Quality:%.2f\n",
                       anchorData[1].range, anchorData[1].fpPower, 
                       anchorData[1].rxPower, anchorData[1].rxQuality);
    }
    
    if(anchorData[2].valid && len < MAX_LEN - 100) {
        len += snprintf(btBuffer + len, MAX_LEN - len,
                       "Anchor_3,Range:%.3f,FP_Power:%.2f,RX_Power:%.2f,RX_Quality:%.2f\n",
                       anchorData[2].range, anchorData[2].fpPower, 
                       anchorData[2].rxPower, anchorData[2].rxQuality);
    }
    
    if(anchorData[3].valid && len < MAX_LEN - 100) {
        len += snprintf(btBuffer + len, MAX_LEN - len,
                       "Anchor_4,Range:%.3f,FP_Power:%.2f,RX_Power:%.2f,RX_Quality:%.2f\n",
                       anchorData[3].range, anchorData[3].fpPower, 
                       anchorData[3].rxPower, anchorData[3].rxQuality);
    }
    
    if(len < MAX_LEN - 30) {
        len += snprintf(btBuffer + len, MAX_LEN - len, 
                       "========================\n");
    }
    
    // Single write operation - much faster and non-blocking
    SerialBT.write((uint8_t*)btBuffer, len);
    #endif
}
 
void loop() {
     // Check for anchor timeout
     if(last_blink_time > 0 && (millis() - last_blink_time) > ANCHOR_TIMEOUT) {
         Serial.println("Anchor timeout - resetting state");
         received_B = false;
         received_C = false;
         last_blink_time = 0;
     }
     
     if(DW1000NgRTLS::receiveFrame()){
         size_t recv_len = DW1000Ng::getReceivedDataLength();
         
         // Fixed size buffer to prevent stack overflow
         byte recv_data[MAX_FRAME_SIZE];
         if(recv_len > MAX_FRAME_SIZE) {
             Serial.println("Error: Frame too large");
             return;
         }
         DW1000Ng::getReceivedData(recv_data, recv_len);

         // Check minimum length before accessing array
         if(recv_len < 1) return;

         if(recv_data[0] == BLINK) {
             // Check minimum BLINK frame size
             if(recv_len < 3) {
                 Serial.println("Error: BLINK frame too short");
                 return;
             }
             
             // Reset all anchor data at start of new ranging cycle
             for(int i = 0; i < 4; i++) {
                 anchorData[i].valid = false;
             }
             
             // Start timeout timer
             last_blink_time = millis();
             
             DW1000NgRTLS::transmitRangingInitiation(&recv_data[2], tag_shortAddress);
             DW1000NgRTLS::waitForTransmission();

             RangeAcceptResult result = DW1000NgRTLS::anchorRangeAccept(NextActivity::RANGING_CONFIRM, next_anchor);
             if(!result.success) return;
             range_self = result.range;
 
             // Store Anchor A (self) data
             anchorData[0].range = range_self;
             anchorData[0].rxPower = DW1000Ng::getReceivePower();
             anchorData[0].fpPower = DW1000Ng::getFirstPathPower();
             anchorData[0].rxQuality = DW1000Ng::getReceiveQuality();
             anchorData[0].valid = true;
 
            Serial.print("Range: ");
            Serial.print(range_self, 3);
            Serial.print(" m\t RX power: ");
            Serial.print(DW1000Ng::getReceivePower(), 2);
            Serial.print(" dBm\t FP power: ");
            Serial.print(DW1000Ng::getFirstPathPower(), 2);
            Serial.println(" dBm");

        } else if(recv_len >= 12 && recv_data[9] == 0x60) {
            // Check minimum ranging report size (need at least 12 bytes for range data)
            // Extended ranging report: range(2) + fpPower(4) + rxPower(4) + rxQuality(4) = 14 bytes
            double range = static_cast<double>(DW1000NgUtils::bytesAsValue(&recv_data[10],2) / 1000.0);
            
            // Extract additional data from extended report
            float fpPower = 0.0, rxPower = 0.0, rxQuality = 0.0;
            if(recv_len >= 24) { // Check if extended report
                memcpy(&fpPower, &recv_data[12], 4);
                memcpy(&rxPower, &recv_data[16], 4);
                memcpy(&rxQuality, &recv_data[20], 4);
            }
            
            Serial.print("Range from: ");
            Serial.print(recv_data[7]);
            Serial.print(" = ");
            Serial.print(range, 3);
            Serial.print(" m, FP: ");
            Serial.print(fpPower, 2);
            Serial.print(" dBm, RX: ");
            Serial.print(rxPower, 2);
            Serial.print(" dBm, Q: ");
            Serial.println(rxQuality, 2);
            
            if(received_B == false && recv_data[7] == anchor_b[0] && recv_data[8] == anchor_b[1]) {
                range_B = range;
                // Store Anchor B data with extended information
                anchorData[1].range = range_B;
                anchorData[1].rxPower = rxPower;
                anchorData[1].fpPower = fpPower;
                anchorData[1].rxQuality = rxQuality;
                anchorData[1].valid = true;
                received_B = true;
            } else if(received_B == true && received_C == false && recv_data[7] == anchor_c[0] && recv_data[8] == anchor_c[1]){
                range_C = range;
                // Store Anchor C data with extended information
                anchorData[2].range = range_C;
                anchorData[2].rxPower = rxPower;
                anchorData[2].fpPower = fpPower;
                anchorData[2].rxQuality = rxQuality;
                anchorData[2].valid = true;
                received_C = true;
            } else if(received_B == true && received_C == true && recv_data[7] == anchor_d[0] && recv_data[8] == anchor_d[1]){
                range_D = range;
                // Store Anchor D data with extended information
                anchorData[3].range = range_D;
                anchorData[3].rxPower = rxPower;
                anchorData[3].fpPower = fpPower;
                anchorData[3].rxQuality = rxQuality;
                anchorData[3].valid = true;
                
                Serial.println("=== All 4 Anchor Ranges ===");
                Serial.print("Anchor A (Main): "); Serial.print(range_self, 3); Serial.println(" m");
                Serial.print("Anchor B: "); Serial.print(range_B, 3); Serial.println(" m");
                Serial.print("Anchor C: "); Serial.print(range_C, 3); Serial.println(" m");
                Serial.print("Anchor D: "); Serial.print(range_D, 3); Serial.println(" m");
                Serial.println("===========================");
                
                // Check if all ranges are valid (not zero)
                if(range_self > 0.0 && range_B > 0.0 && range_C > 0.0 && range_D > 0.0) {
                    // Send all data via Bluetooth only if all ranges are valid
                    sendBluetoothData();
                    Serial.println("Data sent via Bluetooth");
                } else {
                    Serial.println("Invalid range detected (zero), skipping Bluetooth transmission");
                }
                
                // Reset state for next cycle
                received_B = false;
                received_C = false;
                last_blink_time = 0;  // Clear timeout timer
            } else {
                received_B = false;
                received_C = false;
            }
         }
     }
 
     
 }