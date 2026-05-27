// === インクルードと定義 ===
#include <BLEDevice.h>
#include <BLEUtils.h>
#include <BLEClient.h>
#include <BluetoothSerial.h>
#include <LiquidCrystal_I2C.h>

#include <Wire.h>
#include <ICM20948_WE.h>

#define BUTTON_PIN 18
#define HeightAddress 0x70
#define RangeCommand 0x51
#define LcdAddress 0x27
#define SERVICE_UUID      "12345678-1234-1234-1234-123456789abc"
#define CHARACTERISTIC_UUID "abcdefab-cdef-abcd-efab-cdef12345678"
#define PERIPHERAL_MAC "AC:15:18:EB:3E:92"

#define LED_PIN 13

BluetoothSerial SerialBT;
BLEClient *pClient = nullptr;
BLERemoteCharacteristic *pRemoteCharacteristic = nullptr;
bool connected = false;
bool reconnecting = false;
int reconnectAttempts = 0;
#define MAX_RECONNECT_ATTEMPTS 5

LiquidCrystal_I2C lcd(0x27, 16, 2);
int first = 1;

// === 高度用変数 ===
float heightlog = 0;
unsigned long lastUpdateTime = 0;

// === 関数群 ===
void connectToPeripheral() {
    Serial.println("Attempting to connect to peripheral...");
    if (pClient == nullptr) {
        pClient = BLEDevice::createClient();
    }

    if (pClient->connect(BLEAddress(PERIPHERAL_MAC))) {
        Serial.println("Connected to peripheral");
        connected = true;
        reconnecting = false;
        reconnectAttempts = 0;

        BLERemoteService *pService = pClient->getService(SERVICE_UUID);
        if (pService) {
            pRemoteCharacteristic = pService->getCharacteristic(CHARACTERISTIC_UUID);
            if (pRemoteCharacteristic) {
                Serial.println("Characteristic found, ready for polling.");
            } else {
                Serial.println("Characteristic not found.");
            }
        } else {
            Serial.println("Service not found.");
        }
    } else {
        Serial.println("Connection failed.");
        connected = false;
        reconnecting = true;
    }
}

// === セットアップ ===
void setup() {
    Wire.begin(21, 22);
    Wire.setClock(100000);
    delay(1000);
    lcd.init();
    lcd.clear();
    lcd.setCursor(0, 0);
    lcd.print("LCD ready");
    delay(1000);
    lcd.noBacklight();

    Serial.begin(115200);
    SerialBT.begin("BTC Device 01");

    BLEDevice::init("ESP32 Master");
    connectToPeripheral();

    pinMode(BUTTON_PIN, INPUT_PULLUP);
    lastUpdateTime = millis();

}
int e_value=0;
int r_value=0;
float s_value=0;
int x_value;
int y_value;
int z_value;
// === ループ処理 ===
void loop() {
    static unsigned long lastSendTime = 0;
    unsigned long currentMillis = millis();

    // 再接続処理
    if (!connected) {
        if (reconnecting) {
            if (reconnectAttempts < MAX_RECONNECT_ATTEMPTS) {
                Serial.println("Reconnecting...");
                connectToPeripheral();
                reconnectAttempts++;
                delay(1000);
            } else {
                Serial.println("Max attempts reached. Waiting...");
                delay(5000);
                reconnectAttempts = 0;
            }
        }
    }

    // 切断検出
    if (connected && pRemoteCharacteristic && !pClient->isConnected()) {
        Serial.println("Disconnected.");
        connected = false;
        reconnecting = true;
        first = 1;
        lcd.noBacklight();
    }

    // 高度読み取り
    Wire.beginTransmission(HeightAddress);
    Wire.write(RangeCommand);
    Wire.endTransmission();
    delay(100);
    Wire.requestFrom(HeightAddress, 2);
    float height = (Wire.read() << 8 | Wire.read())/100.0;
    if (height <= 0.4) height = (height - 0.2) * 2;
    if (height >= 7.65 && heightlog <= 5.00) height = 0.0;
    heightlog = height;

    // BLE データ取得（通知ではなくloop内で取得）
    if (connected && pRemoteCharacteristic && pRemoteCharacteristic->canRead()) {
        String value = pRemoteCharacteristic->readValue();
        String receivedData = String(value.c_str());
        int e_index = receivedData.indexOf('e');
        int r_index = receivedData.indexOf('r');
        int s_index = receivedData.indexOf('s');
        int x_index = receivedData.indexOf('x');
        int y_index = receivedData.indexOf('y');
        int z_index = receivedData.indexOf('z');

        if (e_index != -1 && r_index != -1 && s_index != -1 && x_index != -1 && y_index != -1 && z_index != -1) {
            e_value = -receivedData.substring(0, e_index).toInt();
            r_value = -receivedData.substring(e_index + 1, r_index).toInt();
            s_value = receivedData.substring(r_index + 1, s_index).toFloat();
            x_value = receivedData.substring(s_index + 1, x_index).toInt();
            y_value = receivedData.substring(x_index + 1, y_index).toInt();
            z_value = receivedData.substring(y_index + 1, z_index).toInt();
            
            lcd.backlight();
            lcd.clear();
            lcd.setCursor(0, 0);
            lcd.printf("E:%+03d R:%+03d", e_value, r_value);
            lcd.setCursor(0, 1);
            lcd.print("S:");
            lcd.print(s_value,1);
            lcd.print(" H:");
            lcd.print(height,1); 
        }
         
        if (SerialBT.connected() && currentMillis - lastSendTime >= 100) {
                String dataToSend = "A:" + String(s_value) + ",B:" + String(e_value) + ",C:" + String(r_value) + "\n";
                SerialBT.print(dataToSend);
                Serial.print(dataToSend);
                dataToSend = "H:" + String(height) + ",O:" + String(x_value) + ",P:" + String(y_value) + ",Q:" + String(z_value) + "\n";
                SerialBT.print(dataToSend);
                Serial.println(dataToSend);
                lastSendTime = currentMillis;
        }
    }
}
