#include "esp_adc_cal.h"
#include <BLEDevice.h>
#include <BLEUtils.h>
#include <BLEServer.h>
#include <BLE2902.h>
#include "esp_bt_device.h"
#include <freertos/FreeRTOS.h>
#include <freertos/task.h>

#include <WiFiManager.h> //https://github.com/tzapu/WiFiManager

#include <Wire.h>

#include "ICM_20948.h" // Click here to get the library: http://librarymanager/All#SparkFun_ICM_20948_IMU

ICM_20948_I2C myICM; // Otherwise create an ICM_20948_I2C object

#define AD0_VAL 0

String response;

#include <ESPmDNS.h>
#include <WiFiUdp.h>
#include <ArduinoOTA.h>
#include <EEPROM.h>

// define the number of bytes you want to access
#define EEPROM_SIZE 1

unsigned long lastTime = 0;
unsigned long timerDelay = 1000 * 60 * 2; // Timer set to 2 minutes

String batt_level = "";
bool displayOn = true;

bool otaMode = false;

int writeCount = 0;

boolean start = false;

BLECharacteristic *pCharacteristic;

struct Quat
{
    float x;
    float y;
    float z;
    float w;
} quat;
struct Euler
{
    float x;
    float y;
    float z;
} euler;
char buff[256];
bool rtcIrq = false;
bool initial = 1;
bool otaStart = false;

int fps = 37;
float batt_v = 0.0;
#include "esp_adc_cal.h"
#define BAT_ADC 2

uint8_t func_select = 0;
uint8_t omm = 99;
uint8_t xcolon = 0;
uint32_t targetTime = 0; // for next 1 second timeout
uint32_t colour = 0;
int vref = 1100;

bool pressed = false;
uint32_t pressedTime = 0;
bool charge_indication = false;

uint8_t hh, mm, ss;
String mac_address;
int pacnum = 0;

#define TP_PIN_PIN 33
#define I2C_SDA_PIN 21
#define I2C_SCL_PIN 22
#define IMU_INT_PIN 38
#define RTC_INT_PIN 34
#define BATT_ADC_PIN 35
#define VBUS_PIN 36
#define TP_PWR_PIN 25
#define LED_PIN 4
#define CHARGE_PIN 32

#define BLE_NAME "Quatro" // must match filters name in bluetoothterminal.js- navigator.bluetooth.requestDevice

BLEUUID SERVICE_UUID("6E400001-B5A3-F393-E0A9-E50E24DCCA9E"); // UART service UUID
BLEUUID CHARACTERISTIC_UUID("6E400002-B5A3-F393-E0A9-E50E24DCCA9E");
BLEAdvertising *pAdvertising;



uint32_t readADC_Cal(int ADC_Raw) {
  esp_adc_cal_characteristics_t adc_chars;

  esp_adc_cal_characterize(ADC_UNIT_1, ADC_ATTEN_DB_11, ADC_WIDTH_BIT_12, 1100, &adc_chars);
  return (esp_adc_cal_raw_to_voltage(ADC_Raw, &adc_chars));
}


void setupWiFi()
{
    WiFiManager wifiManager;
    // set callback that gets called when connecting to previous WiFi fails, and enters Access Point mode
    wifiManager.setAPCallback(configModeCallback);
    wifiManager.setBreakAfterConfig(true); // Without this saveConfigCallback does not get fired
    mac_address = WiFi.macAddress();
    Serial.println(mac_address);
    wifiManager.autoConnect(String("Quatro-" + mac_address).c_str());
}

void configModeCallback(WiFiManager *myWiFiManager)
{
    Serial.println("Entered config mode");
    Serial.println(WiFi.softAPIP());
    // if you used auto generated SSID, print it
    Serial.println(myWiFiManager->getConfigPortalSSID());
}

void drawProgressBar(uint16_t x0, uint16_t y0, uint16_t w, uint16_t h, uint8_t percentage, uint16_t frameColor, uint16_t barColor)
{
}

void setupOTA()
{

    // Hostname defaults to esp3232-[MAC]
    ArduinoOTA.setHostname(mac_address.c_str());

    ArduinoOTA.onStart([]()
                       {
              String type;
              if (ArduinoOTA.getCommand() == U_FLASH)
                type = "sketch";
              else  // U_SPIFFS
                type = "filesystem";

              // NOTE: if updating SPIFFS this would be the place to unmount SPIFFS using SPIFFS.end()
              Serial.println("Start updating " + type);
              otaStart = true; })
        .onEnd([]()
               {
      Serial.println("\nEnd");
      delay(500); })
        .onProgress([](unsigned int progress, unsigned int total)
                    {
      // Serial.printf("Progress: %u%%\r", (progress / (total / 100)));
      int percentage = (progress / (total / 100)); })
        .onError([](ota_error_t error)
                 {
      Serial.printf("Error[%u]: ", error);
      if (error == OTA_AUTH_ERROR) Serial.println("Auth Failed");
      else if (error == OTA_BEGIN_ERROR) Serial.println("Begin Failed");
      else if (error == OTA_CONNECT_ERROR) Serial.println("Connect Failed");
      else if (error == OTA_RECEIVE_ERROR) Serial.println("Receive Failed");
      else if (error == OTA_END_ERROR) Serial.println("End Failed");

      delay(3000);
      otaStart = false;
      initial = 1;
      targetTime = millis() + 1000;
      omm = 99; });

    ArduinoOTA.begin();
}

String getVoltage()
{
    uint16_t v = analogRead(BATT_ADC_PIN);
    float battery_voltage = ((float)v / 4095.0) * 2.0 * 3.3 * (vref / 1000.0);
    return String(battery_voltage) + "V";
}

String Bone = "N/A";
bool calibrated = false;

class MyCallbacks : public BLECharacteristicCallbacks
{
    void onWrite(BLECharacteristic *pChar)
    {
        std::string value = pChar->getValue();
        Serial.print("Received Value: ");
        Serial.println(value.c_str());

        if (value == "start")
        {
            if (!start)
            {
            }
        }
        else if (value == "calibrate")
        {
            start = true;
        }
        else if (value == "dispoff")
        {
            if (calibrated)
            {

                displayOn = false;
            }
        }

        else if (value == "restart")
        {
            delay(3000);
            ESP.restart();
        }

        else if (value == "ota")
        {
            EEPROM.write(0, 1);
            EEPROM.commit();
            delay(2000);
            ESP.restart();
        }
        else
        {
            Bone = String(value.c_str());
        }
    }
};

class ServerCallbacks : public BLEServerCallbacks
{
    void onDisconnect(BLEServer *server)
    {
        Serial.print("Disconnected");
        pAdvertising->start();
    }
};

void setup()
{
    Serial.begin(115200);
    EEPROM.begin(EEPROM_SIZE);

    pinMode(LED_PIN, OUTPUT);

    int otaState = EEPROM.read(0);
    Serial.println(otaState);

    if (otaState == 1)
    {
        EEPROM.write(0, 0);
        EEPROM.commit();
        otaMode = true;
        otaStart = true;
        setupWiFi();
        setupOTA();
        return;
    }

    BLEDevice::init(BLE_NAME);
    BLEServer *pServer = BLEDevice::createServer();
    pServer->setCallbacks(new ServerCallbacks());

    BLEService *pService = pServer->createService(SERVICE_UUID);

    pCharacteristic = pService->createCharacteristic(
        CHARACTERISTIC_UUID,
        BLECharacteristic::PROPERTY_READ | BLECharacteristic::PROPERTY_WRITE | BLECharacteristic::PROPERTY_NOTIFY);

    pCharacteristic->setCallbacks(new MyCallbacks());

    pCharacteristic->addDescriptor(new BLE2902());

    mac_address = BLEDevice::getAddress().toString().c_str();

    esp_ble_gap_set_device_name(("Quatro-" + mac_address).c_str());
    esp_bt_dev_set_device_name(("Quatro-" + mac_address).c_str());

    pService->start();

    pAdvertising = pServer->getAdvertising();
    pAdvertising->start();

    delay(2000); // Wait for BNO to boot
    // Start i2c and BNO080
    Wire.flush();       // Reset I2C
    Wire.begin(19, 18); // I2C pins setup
    Wire.setClock(400000);

    bool initialized = false;
    while (!initialized)
    {

        // Initialize the ICM-20948
        // If the DMP is enabled, .begin performs a minimal startup. We need to configure the sample mode etc. manually.

        myICM.begin(Wire, AD0_VAL);

        if (myICM.status != ICM_20948_Stat_Ok)
        {
            Serial.println(F("Trying again..."));
            delay(500);
        }
        else
        {
            initialized = true;
        }
    }

    Serial.println(F("Device connected!"));

    bool success = true; // Use success to show if the DMP configuration was successful

    // Initialize the DMP. initializeDMP is a weak function. You can overwrite it if you want to e.g. to change the sample rate
    success &= (myICM.initializeDMP() == ICM_20948_Stat_Ok);

    // DMP sensor options are defined in ICM_20948_DMP.h
    //    INV_ICM20948_SENSOR_ACCELEROMETER               (16-bit accel)
    //    INV_ICM20948_SENSOR_GYROSCOPE                   (16-bit gyro + 32-bit calibrated gyro)
    //    INV_ICM20948_SENSOR_RAW_ACCELEROMETER           (16-bit accel)
    //    INV_ICM20948_SENSOR_RAW_GYROSCOPE               (16-bit gyro + 32-bit calibrated gyro)
    //    INV_ICM20948_SENSOR_MAGNETIC_FIELD_UNCALIBRATED (16-bit compass)
    //    INV_ICM20948_SENSOR_GYROSCOPE_UNCALIBRATED      (16-bit gyro)
    //    INV_ICM20948_SENSOR_STEP_DETECTOR               (Pedometer Step Detector)
    //    INV_ICM20948_SENSOR_STEP_COUNTER                (Pedometer Step Detector)
    //    INV_ICM20948_SENSOR_GAME_ROTATION_VECTOR        (32-bit 6-axis quaternion)
    //    INV_ICM20948_SENSOR_ROTATION_VECTOR             (32-bit 9-axis quaternion + heading accuracy)
    //    INV_ICM20948_SENSOR_GEOMAGNETIC_ROTATION_VECTOR (32-bit Geomag RV + heading accuracy)
    //    INV_ICM20948_SENSOR_GEOMAGNETIC_FIELD           (32-bit calibrated compass)
    //    INV_ICM20948_SENSOR_GRAVITY                     (32-bit 6-axis quaternion)
    //    INV_ICM20948_SENSOR_LINEAR_ACCELERATION         (16-bit accel + 32-bit 6-axis quaternion)
    //    INV_ICM20948_SENSOR_ORIENTATION                 (32-bit 9-axis quaternion + heading accuracy)

    // Enable the DMP Game Rotation Vector sensor
    success &= (myICM.enableDMPSensor(INV_ICM20948_SENSOR_GAME_ROTATION_VECTOR) == ICM_20948_Stat_Ok);
    // success &= (myICM.enableDMPSensor(INV_ICM20948_SENSOR_GRAVITY) == ICM_20948_Stat_Ok);

    // Enable any additional sensors / features
    // success &= (myICM.enableDMPSensor(INV_ICM20948_SENSOR_RAW_GYROSCOPE) == ICM_20948_Stat_Ok);
    // success &= (myICM.enableDMPSensor(INV_ICM20948_SENSOR_RAW_ACCELEROMETER) == ICM_20948_Stat_Ok);
    // success &= (myICM.enableDMPSensor(INV_ICM20948_SENSOR_MAGNETIC_FIELD_UNCALIBRATED) == ICM_20948_Stat_Ok);

    // Configuring DMP to output data at multiple ODRs:
    // DMP is capable of outputting multiple sensor data at different rates to FIFO.
    // Setting value can be calculated as follows:
    // Value = (DMP running rate / ODR ) - 1
    // E.g. For a 5Hz ODR rate when DMP is running at 55Hz, value = (55/5) - 1 = 10.
    success &= (myICM.setDMPODRrate(DMP_ODR_Reg_Quat6, 0) == ICM_20948_Stat_Ok); // Set to the maximum
    // success &= (myICM.setDMPODRrate(DMP_ODR_Reg_Accel, 0) == ICM_20948_Stat_Ok); // Set to the maximum
    // success &= (myICM.setDMPODRrate(DMP_ODR_Reg_Gyro, 0) == ICM_20948_Stat_Ok); // Set to the maximum
    // success &= (myICM.setDMPODRrate(DMP_ODR_Reg_Gyro_Calibr, 0) == ICM_20948_Stat_Ok); // Set to the maximum
    // success &= (myICM.setDMPODRrate(DMP_ODR_Reg_Cpass, 0) == ICM_20948_Stat_Ok); // Set to the maximum
    // success &= (myICM.setDMPODRrate(DMP_ODR_Reg_Cpass_Calibr, 0) == ICM_20948_Stat_Ok); // Set to the maximum

    // Enable the FIFO
    success &= (myICM.enableFIFO() == ICM_20948_Stat_Ok);

    // Enable the DMP
    success &= (myICM.enableDMP() == ICM_20948_Stat_Ok);

    // Reset DMP
    success &= (myICM.resetDMP() == ICM_20948_Stat_Ok);

    // Reset FIFO
    success &= (myICM.resetFIFO() == ICM_20948_Stat_Ok);

    // Check success
    if (success)
    {
#ifndef QUAT_ANIMATION
        Serial.println(F("DMP enabled!"));
#endif
    }
    else
    {
        Serial.println(F("Enable DMP failed!"));
        Serial.println(F("Please check that you have uncommented line 29 (#define ICM_20948_USE_DMP) in ICM_20948_C.h..."));
        while (1)
            ; // Do nothing more
    }

    xTaskCreatePinnedToCore(
        TaskBluetooth, "TaskBluetooth", 10000, NULL, 1, NULL, 0);

    xTaskCreatePinnedToCore(
        TaskReadICM, "TaskReadICM", 10000, NULL, 1, NULL, 1);
}

void loop()
{
    // Nothing to do in the loop, all tasks are handled by FreeRTOS tasks.
}

void TaskReadICM(void *pvParameters)
{
    for (;;)
    {

        static uint32_t prev_ms1 = millis();
        if (millis() > (prev_ms1 + 1000 * fps))
        {
            // read battery every minute
            batt_v = (readADC_Cal(analogRead(BAT_ADC))) * 2;
            prev_ms1 = millis();
        }

        icm_20948_DMP_data_t data;
        myICM.readDMPdataFromFIFO(&data);

        if ((myICM.status == ICM_20948_Stat_Ok) || (myICM.status == ICM_20948_Stat_FIFOMoreDataAvail)) // Was valid data available?
        {
            // Serial.print(F("Received data! Header: 0x")); // Print the header in HEX so we can see what data is arriving in the FIFO
            // if ( data.header < 0x1000) Serial.print( "0" ); // Pad the zeros
            // if ( data.header < 0x100) Serial.print( "0" );
            // if ( data.header < 0x10) Serial.print( "0" );
            // Serial.println( data.header, HEX );

            if ((data.header & DMP_header_bitmap_Quat6) > 0) // We have asked for GRV data so we should receive Quat6
            {
                // Q0 value is computed from this equation: Q0^2 + Q1^2 + Q2^2 + Q3^2 = 1.
                // In case of drift, the sum will not add to 1, therefore, quaternion data need to be corrected with right bias values.
                // The quaternion data is scaled by 2^30.

                // Serial.printf("Quat6 data is: Q1:%ld Q2:%ld Q3:%ld\r\n", data.Quat6.Data.Q1, data.Quat6.Data.Q2, data.Quat6.Data.Q3);

                // Scale to +/- 1
                double q1 = ((double)data.Quat6.Data.Q1) / 1073741824.0; // Convert to double. Divide by 2^30
                double q2 = ((double)data.Quat6.Data.Q2) / 1073741824.0; // Convert to double. Divide by 2^30
                double q3 = ((double)data.Quat6.Data.Q3) / 1073741824.0; // Convert to double. Divide by 2^30

                // Convert the quaternions to Euler angles (roll, pitch, yaw)
                // https://en.wikipedia.org/w/index.php?title=Conversion_between_quaternions_and_Euler_angles&section=8#Source_code_2

                double q0 = sqrt(1.0 - ((q1 * q1) + (q2 * q2) + (q3 * q3)));

                double q2sqr = q2 * q2;

                // roll (x-axis rotation)
                double t0 = +2.0 * (q0 * q1 + q2 * q3);
                double t1 = +1.0 - 2.0 * (q1 * q1 + q2sqr);
                double roll = atan2(t0, t1) * 180.0 / PI;

                // pitch (y-axis rotation)
                double t2 = +2.0 * (q0 * q2 - q3 * q1);
                t2 = t2 > 1.0 ? 1.0 : t2;
                t2 = t2 < -1.0 ? -1.0 : t2;
                double pitch = asin(t2) * 180.0 / PI;

                // yaw (z-axis rotation)
                double t3 = +2.0 * (q0 * q3 + q1 * q2);
                double t4 = +1.0 - 2.0 * (q2sqr + q3 * q3);
                double yaw = atan2(t3, t4) * 180.0 / PI;

                Serial.print(q0, 3);
                Serial.print(" ");
                Serial.print(q1, 3);
                Serial.print(" ");
                Serial.print(q2, 3);
                Serial.print(" ");
                Serial.print(q3, 3);
                Serial.println();

                quat.w = q0;
                quat.x = q1;
                quat.y = q2;
                quat.z = q3;
            }
        }

        if (myICM.status != ICM_20948_Stat_FIFOMoreDataAvail) // If more data is available then we should read it right away - and not delay
        {
            delay(10);
        }

        // vTaskDelay(1);  // one tick delay (15ms) in between reads for stability
    }
}


void TaskBluetooth(void *pvParameters)
{
  int count = 0;
    for (;;)
    {
        static uint32_t prev_ms_ble = millis();
        if (millis() > prev_ms_ble + 1000 / fps)
        {
            prev_ms_ble = millis();
            String url = mac_address + " " + String(quat.x, 4) + " " + String(quat.y, 4) + " " + String(quat.z, 4) + " " + String(quat.w, 4) + " " + String(count++) + " " + String(batt_v);
            pCharacteristic->setValue(url.c_str());
            pCharacteristic->notify();
        }
        vTaskDelay(1);
    }
}
