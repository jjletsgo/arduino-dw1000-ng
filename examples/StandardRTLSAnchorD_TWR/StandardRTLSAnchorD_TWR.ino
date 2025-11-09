/*
   This example code is in the Public Domain (or CC0 licensed, at your option.)

   Unless required by applicable law or agreed to in writing, this
   software is distributed on an "AS IS" BASIS, WITHOUT WARRANTIES OR
   CONDITIONS OF ANY KIND, either express or implied.
*/

/* 
 * StandardRTLSAnchorD_TWR.ino
 * 
 * This is an example slave anchor in a RTLS using two way ranging ISO/IEC 24730-62_2013 messages
 */

 #include <DW1000Ng.hpp>
 #include <DW1000NgUtils.hpp>
 #include <DW1000NgRanging.hpp>
 #include <DW1000NgRTLS.hpp>
 
 // connection pins
 const uint8_t PIN_SCK = 18;
 const uint8_t PIN_MOSI = 23;
 const uint8_t PIN_MISO = 19;
 const uint8_t PIN_SS = 4;
 const uint8_t PIN_RST = 15; 
 const uint8_t PIN_IRQ = 17;
 
 // Extended Unique Identifier register. 64-bit device identifier. Register file: 0x01
 const char EUI[] = "AA:BB:CC:DD:EE:FF:00:04"; 
 
 byte main_anchor_address[] = {0x01, 0x00};
 
uint16_t blink_rate = 20;

double range_self = 0.0;  // Initialize to prevent sending garbage values
 
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
     false
 };
 
 void setup() {
     // DEBUG monitoring
     Serial.begin(115200);
     Serial.println(F("### arduino-DW1000Ng-ranging-anchor-D ###"));
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
     DW1000Ng::setDeviceAddress(4);
     
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
 
void transmitRangeReport() {
    // Extended ranging report with additional data
    byte rangingReport[] = {DATA, SHORT_SRC_AND_DEST, DW1000NgRTLS::increaseSequenceNumber(), 0,0, 0,0, 0,0, 0x60, 0,0, 0,0,0,0, 0,0,0,0, 0,0,0,0 };
    DW1000Ng::getNetworkId(&rangingReport[3]);
    memcpy(&rangingReport[5], main_anchor_address, 2);
    DW1000Ng::getDeviceAddress(&rangingReport[7]);
    DW1000NgUtils::writeValueToBytes(&rangingReport[10], static_cast<uint16_t>((range_self*1000)), 2);
    
    // Add FP Power, RX Power, RX Quality
    float fpPower = DW1000Ng::getFirstPathPower();
    float rxPower = DW1000Ng::getReceivePower();
    float rxQuality = DW1000Ng::getReceiveQuality();
    memcpy(&rangingReport[12], &fpPower, 4);
    memcpy(&rangingReport[16], &rxPower, 4);
    memcpy(&rangingReport[20], &rxQuality, 4);
    
    DW1000Ng::setTransmitData(rangingReport, sizeof(rangingReport));
    DW1000Ng::startTransmit();
}
  
 void loop() {
     RangeAcceptResult result = DW1000NgRTLS::anchorRangeAccept(NextActivity::ACTIVITY_FINISHED, blink_rate);
     if(result.success) {
        delay(5); // Tweak based on your hardware
        range_self = result.range;
        
        // Only transmit if range is valid (not zero)
        if(range_self > 0.0) {
            transmitRangeReport();
            DW1000NgRTLS::waitForTransmission();  // Wait for transmission to complete

            Serial.print("Range: ");
            Serial.print(range_self, 3);
            Serial.print(" m\t RX power: ");
            Serial.print(DW1000Ng::getReceivePower(), 2);
            Serial.print(" dBm\t FP power: ");
            Serial.print(DW1000Ng::getFirstPathPower(), 2);
            Serial.println(" dBm");
        } else {
            Serial.println("Invalid range (zero), skipping report transmission");
        }
     }
}
 
 