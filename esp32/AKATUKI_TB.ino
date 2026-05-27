#include <Wire.h>
#include <ICM20948_WE.h>
#include <EEPROM.h>
#include "C:/Users/tttt1/AppData/Local/Arduino15/packages/esp32/hardware/esp32/3.0.4/libraries/BLE/src/BLEDevice.h"
#include <BLEUtils.h>
#include <BLEServer.h>
#include <math.h> // fmodf のために必要

#define SDP810_ADDR 0x25
#define ICM20948_ADDR 0x68
#define SDA_PIN 21 // SDA ピン
#define SCL_PIN 22 // SCL ピン
#define e_pin 25
#define r_pin 26
#define switch1 32
#define switch2 33
#define SERVICE_UUID "12345678-1234-1234-1234-123456789abc"     // サービスUUID
#define CHARACTERISTIC_UUID "abcdefab-cdef-abcd-efab-cdef12345678" // キャラクタリスティックUUID

// 割り込み
volatile unsigned long switch1PressTime = 0; // スイッチが押された時刻を記録
volatile unsigned long switch2PressTime = 0;
volatile bool switch1Flag = false; // スイッチが押されたことを示すフラグ
volatile bool switch2Flag = false;

// 同時押し判定の許容時間 (ms)
const unsigned long simultaneousPressWindow = 100; // この時間差以内なら同時押しとみなす
const unsigned long doubleClickInterval = 400;     // ダブルクリックの判定時間 (EEPROM保存用)
const unsigned long debounceDelay = 50;            // チャタリング防止のための最小間隔

// 舵角
int e_basis;
int r_basis;

// 機速
const float AIR_DENSITY = 1.225; // 空気密度 (kg/m³)

// 姿勢角
ICM20948_WE myIMU = ICM20948_WE(ICM20948_ADDR);
float roll = 0.0, pitch = 0.0, yaw = 0.0;
unsigned long lastUpdateTime = 0;
const float alpha = 0.85; // Complementary filter rate（少し小さくして応答を速くする）
bool imuReady = true;     // 初期状態では正常と仮定
float initialRoll = 0.0, initialPitch = 0.0;
float rollOffset = 0.0;
int x_log=0;
int y_log=0;
float magMinX =   1000, magMaxX = -1000;
float magMinY =   1000, magMaxY = -1000;
bool calibrateMag = false;  


void resetIMU() {
  // センサ初期化処理
  myIMU.autoOffsets(); // センサのオフセットを再計算
  Serial.println("IMU Resetting...");
  delay(1000); // リセット処理の待機

  // 初期角度を再設定
  myIMU.readSensor();
  xyzFloat accRaw;
  myIMU.getAccRawValues(&accRaw);
  accRaw.x = -accRaw.x;
  accRaw.y = -accRaw.y;
  accRaw.z = -accRaw.z;

  // 加速度センサーから初期のroll角を計算
  float accInitialRoll = atan2(accRaw.y, accRaw.z) * 180.0 / PI;

  // rollOffset を計算: 今のaccInitialRollが0になるように調整
  rollOffset = accInitialRoll;

  // roll と initialRoll を 0 に設定
  roll = 0.0;
  initialRoll = 0.0; // xの基準も0に

  // pitch の初期化はこれまで通り
  pitch = atan2(-accRaw.x, sqrt(accRaw.y * accRaw.y + accRaw.z * accRaw.z)) * 180.0 / PI;
  initialPitch = pitch; // pitch の初期値はそのまま使用

  yaw = 0.0;

  Serial.println("IMU Reset Complete and Initial Angles Set.");
}

// bluetooth
BLEServer *pServer = NULL;
BLECharacteristic *pCharacteristic = NULL;
bool deviceConnected = false;

// サーバーコールバッククラス
class MyServerCallbacks : public BLEServerCallbacks {
  void onConnect(BLEServer *pServer) {
    deviceConnected = true;
    Serial.println("Device connected");
  }

  void onDisconnect(BLEServer *pServer) {
    deviceConnected = false;
    Serial.println("Device disconnected");
    BLEDevice::startAdvertising(); // 切断時に再度アドバタイズを開始
  }
};

void setup() {
  Serial.begin(115200);

  // bluetooth
  BLEDevice::init("ESP32_Peripheral");
  pServer = BLEDevice::createServer();
  pServer->setCallbacks(new MyServerCallbacks());

  BLEService *pService = pServer->createService(SERVICE_UUID);

  pCharacteristic = pService->createCharacteristic(
      CHARACTERISTIC_UUID,
      BLECharacteristic::PROPERTY_READ | BLECharacteristic::PROPERTY_NOTIFY);

  pCharacteristic->setValue("Initial Data");
  pService->start();

  BLEAdvertising *pAdvertising = BLEDevice::getAdvertising();
  pAdvertising->addServiceUUID(SERVICE_UUID);
  pAdvertising->start();

  // 機速
  Wire.begin(SDA_PIN, SCL_PIN);
  Wire.setClock(100000);
  delay(500);

  // 姿勢角
  lastUpdateTime = millis();
  myIMU.autoOffsets();
  myIMU.setGyrRange(ICM20948_GYRO_RANGE_250);
  myIMU.setAccRange(ICM20948_ACC_RANGE_4G);
  myIMU.setGyrDLPF(ICM20948_DLPF_6);
  myIMU.setAccDLPF(ICM20948_DLPF_6);
  myIMU.initMagnetometer();
  myIMU.readSensor();
  xyzFloat accRaw;
  myIMU.getAccRawValues(&accRaw);
  accRaw.x = -accRaw.x;
  accRaw.y = -accRaw.y;
  accRaw.z = -accRaw.z;

  yaw = 0.0;

  // 加速度センサーから初期のroll角を計算
  float accInitialRoll = atan2(accRaw.y, accRaw.z) * 180.0 / PI;

  // rollOffset を計算: 今のaccInitialRollが0になるように調整
  rollOffset = accInitialRoll;

  // roll と initialRoll を 0 に設定
  roll = 0.0;
  initialRoll = 0.0; // xの基準も0に
  
  // pitch の初期化はこれまで通り
  pitch = atan2(-accRaw.x, sqrt(accRaw.y * accRaw.y + accRaw.z * accRaw.z)) * 180.0 / PI;
  initialPitch = pitch; // pitch の初期値はそのまま使用

  Serial.print("Initial Roll (after offset adjustment): ");
  

  // 舵角
  pinMode(switch1, INPUT_PULLUP);
  pinMode(switch2, INPUT_PULLUP);
  // FALLINGで割り込みを設定し、押された瞬間を捉える
  attachInterrupt(digitalPinToInterrupt(switch1), [] {
    unsigned long currentMicros = micros();
    // 最後に割り込みが発生した時刻から一定時間経過しているかチェックしてチャタリングを防止
    if (currentMicros - switch1PressTime > debounceDelay * 1000) {
      switch1PressTime = currentMicros;
      switch1Flag = true;
    }
  }, FALLING);
  attachInterrupt(digitalPinToInterrupt(switch2), [] {
    unsigned long currentMicros = micros();
    // 最後に割り込みが発生した時刻から一定時間経過しているかチェックしてチャタリングを防止
    if (currentMicros - switch2PressTime > debounceDelay * 1000) {
      switch2PressTime = currentMicros;
      switch2Flag = true;
    }
  }, FALLING);


  EEPROM.begin(16);
  EEPROM.get(0, e_basis);
  EEPROM.get(4, r_basis);
  Serial.print("Loaded e_basis: ");
  Serial.print(e_basis);
  Serial.print(", r_basis: ");
  Serial.println(r_basis);
}

void loop() {
  unsigned long currentTime = millis();

  // --- スイッチの処理 ---
  static unsigned long lastClick1Time = 0; // EEPROM保存用のダブルクリック判定に使用
  static unsigned long lastClick2Time = 0;

  static bool switch1DoubleClickReady = false; // EEPROM保存用のフラグ
  static bool switch2DoubleClickReady = false;

  // --- 同時押しによるIMUリセット判定 ---
  if (switch1Flag && switch2Flag) {
    // 両方のフラグが立っている場合、時刻差をチェック
    // micros() で取得した時刻の差を計算
    if (abs((long)(switch1PressTime - switch2PressTime)) < (simultaneousPressWindow * 1000)) {
      Serial.println("Reset Condition Detected: Both Switches Pressed Simultaneously!");
      resetIMU();
      // リセット後はフラグをクリアして、次回の同時押しに備える
      switch1Flag = false;
      switch2Flag = false;
      // EEPROM保存用のダブルクリック判定フラグもクリアし、誤動作を防ぐ
      switch1DoubleClickReady = false;
      switch2DoubleClickReady = false;
      lastClick1Time = 0; // 次のクリック検出に備える
      lastClick2Time = 0;
    } else {
      // 同時押しとみなせない場合は、どちらかのフラグをクリア
      // 厳密な同時押しではないが、どちらかが先に押されたと判断
      if (switch1PressTime > switch2PressTime) {
        switch2Flag = false; // switch1 が後に押されたので、switch2 のフラグをクリア
      } else {
        switch1Flag = false; // switch2 が後に押されたので、switch1 のフラグをクリア
      }
    }
  }

  // --- 個別のスイッチ処理（EEPROM保存用、ダブルクリック判定） ---
  if (switch1Flag) {
    switch1Flag = false; // フラグをクリア

    if (currentTime - lastClick1Time < doubleClickInterval && lastClick1Time != 0) {
      switch1DoubleClickReady = true;
      Serial.println("Switch1 Double Click Detected for E_basis save.");
    }
    lastClick1Time = currentTime;
  }

  if (switch2Flag) {
    switch2Flag = false; // フラグをクリア

    if (currentTime - lastClick2Time < doubleClickInterval && lastClick2Time != 0) {
      switch2DoubleClickReady = true;
      Serial.println("Switch2 Double Click Detected for R_basis save.");
    }
    lastClick2Time = currentTime;
  }

  if (switch1DoubleClickReady) {
    Serial.println("Executing E_basis save.");
    // 舵角の平均値を算出
    int e_sum = 0;
    int e_maxdata = 0;
    int e_mindata = 4096;
    for (int i = 0; i < 10; i++) {
      int current_e_data = analogRead(e_pin);
      if (e_maxdata < current_e_data) e_maxdata = current_e_data;
      if (e_mindata > current_e_data) e_mindata = current_e_data;
      e_sum += current_e_data;
      delay(10);
    }
    e_sum -= e_maxdata + e_mindata;
    e_basis = e_sum / 8; // maxとminを除外した8回
    
    EEPROM.put(0, e_basis);
    EEPROM.commit();
    Serial.print("e_basis updated to: ");
    Serial.println(e_basis);
    switch1DoubleClickReady = false;
  }

  if (switch2DoubleClickReady) {
    Serial.println("Executing R_basis save.");
    // 舵角の平均値を算出
    int r_sum = 0;
    int r_maxdata = 0;
    int r_mindata = 4096;
    for (int i = 0; i < 20; i++) {
      int current_r_data = analogRead(r_pin);
      if (r_maxdata < current_r_data) r_maxdata = current_r_data;
      if (r_mindata > current_r_data) r_mindata = current_r_data;
      r_sum += current_r_data;
      delay(5);
    }
    r_sum -= r_maxdata + r_mindata;
    r_basis = r_sum / 18; // maxとminを除外した8回

    EEPROM.put(4, r_basis); // r_basis はアドレス4に保存
    EEPROM.commit();
    Serial.print("r_basis updated to: ");
    Serial.println(r_basis);
    switch2DoubleClickReady = false;
  }

  // --- 機速 ---
  Wire.beginTransmission(SDP810_ADDR);
  Wire.write(0x36);
  Wire.write(0x15);
  Wire.endTransmission();
  delay(10); // センサーがデータを用意するのを待つ

  Wire.requestFrom(SDP810_ADDR, 9);
    int16_t rawPressure = (Wire.read() << 8) | Wire.read();
    Wire.read(); // CRCバイトを読み飛ばす
    float pressurePa = rawPressure / 60.0;
    if (pressurePa < 0) {
      pressurePa = 0;
    }
    float speed = sqrt(2 * pressurePa / AIR_DENSITY);

    // --- 姿勢角 ---
  myIMU.readSensor();

  xyzFloat accRaw, gyrRaw, magRaw;
  myIMU.getAccRawValues(&accRaw);
  myIMU.getGyrValues(&gyrRaw);
  myIMU.getMagValues(&magRaw);

  float dt = (currentTime - lastUpdateTime) / 1000.0;
  lastUpdateTime = currentTime;

  accRaw.x = -accRaw.x;
  accRaw.y = -accRaw.y;
  accRaw.z = -accRaw.z;
  gyrRaw.x = -gyrRaw.x;
  gyrRaw.y = -gyrRaw.y;
  gyrRaw.z = -gyrRaw.z;

  // --- 磁気センサキャリブレーション（回転しながら実行） ---
  if (calibrateMag) {
    magMinX = min(magMinX, magRaw.x);
    magMaxX = max(magMaxX, magRaw.x);
    magMinY = min(magMinY, magRaw.y);
    magMaxY = max(magMaxY, magRaw.y);
  }

  // ハードアイアン補正
  float magOffsetX = (magMaxX + magMinX) / 2.0;
  float magOffsetY = (magMaxY + magMinY) / 2.0;
  float correctedMagX = magRaw.x - magOffsetX;
  float correctedMagY = magRaw.y - magOffsetY;

  // --- 地磁気でYaw計算 ---
  float pitchRad = pitch * PI / 180.0;
  float rollRad = roll * PI / 180.0;

  // チルト補正付き磁気方位角の計算
  float xh = magRaw.x * cos(pitchRad) + magRaw.z * sin(pitchRad);
  float yh = magRaw.x * sin(rollRad) * sin(pitchRad) + magRaw.y * cos(rollRad) - magRaw.z * sin(rollRad) * cos(pitchRad);

  float heading = atan2(yh, xh) * 180.0 / PI;
  if (heading < 0) heading += 360;   // 方角が負の場合は360度を加算

  // --- 地磁気に基づくyawの更新 ---
  yaw = 0.90 * yaw + 0.10 * heading;
  int z = yaw;

  float accRoll = atan2(accRaw.y, accRaw.z) * 180.0 / PI;
  float accPitch = atan2(-accRaw.x, sqrt(accRaw.y * accRaw.y + accRaw.z * accRaw.z)) * 180.0 / PI;

  // ここでジャイロによる角度更新
  roll += gyrRaw.x * dt;
  pitch += gyrRaw.y * dt;

  // ****** 修正箇所ここから ******
  // ロール角の連続性補正
  // ジャイロで予測されたrollと、加速度計で算出されたrollの間に
  // 180度以上の大きな乖離がある場合、それを修正します。
  // (accRoll - rollOffset) は、現在のaccRollの、初期姿勢からの相対的な値。
  float errorRoll = (accRoll - rollOffset) - roll; 

  // 角度の差分を -180 から 180 の範囲に正規化する
  // これにより、例えば 170度と-170度の差が340度ではなく、-20度（または20度）と認識される
  while (errorRoll > 180) errorRoll -= 360;
  while (errorRoll < -180) errorRoll += 360;

  // 相補フィルター (補正された accRoll を使用)
  roll = alpha * roll + (1.0 - alpha) * (roll + errorRoll); // 相補フィルターの適用方法を少し変更
  // ****** 修正箇所ここまで ******

  pitch = alpha * pitch + (1.0 - alpha) * accPitch; // pitchはそのまま

  // ロール角とピッチ角の差分を直接整数にキャスト
  // xとyの値の範囲を、初期値を0として、-180から180になるように正規化
  float rawX = roll - initialRoll; // initialRoll は 0 なので rawX = roll と同じ
  float rawY = pitch - initialPitch;

  int x = (int)round(rawX); // 正規化して丸める
  int y = - (int)round(rawY); // 正規化して丸める

      // --- 舵角の測定 (毎回ループで実行) ---
    int e_sum = 0;
    int r_sum = 0;
    int e_maxdata = 0;
    int e_mindata = 4096;
    int r_maxdata = 0;
    int r_mindata = 4096;

      // 舵角はリアルタイム性を考慮し、ここで測定
    for (int i = 0; i < 10; i++) { // 4回測定して平均を取る
    if(e_maxdata < analogRead(e_pin)) e_maxdata = analogRead(e_pin);
    if(e_mindata > analogRead(e_pin)) e_mindata = analogRead(e_pin);
    if(r_maxdata < analogRead(r_pin)) r_maxdata = analogRead(r_pin);
    if(r_mindata > analogRead(r_pin)) r_mindata = analogRead(r_pin);
        e_sum += analogRead(e_pin);
        r_sum += analogRead(r_pin);
        delay(10); // 100ms周期を10回測定する形に
    }
    e_sum -= e_maxdata + e_mindata;
    r_sum -= r_maxdata + r_mindata;

    int e_data = e_sum / 8;//maxとminを除外した8回
    int r_data = r_sum / 8;//maxとminを除外した8回

      int e_value = map(e_data, e_basis - 1082, e_basis + 1070, -90, 90);
      int r_value = map(r_data, r_basis - 1082, r_basis + 1070, -90, 90);

      // --- Bluetooth ---
      if (deviceConnected) {
        String sensorValueStr = String(e_value) + "e" + String(r_value) + "r" + String(speed) + "s" + String(x) + "x" + String(y) + "y" + String(z) + "z";
        pCharacteristic->setValue(sensorValueStr.c_str());
        pCharacteristic->notify(); // 値が更新されたことを通知
      }

      Serial.print("e:");
      Serial.print(e_value);
      Serial.print("r:");
      Serial.print(r_value);
      Serial.print("s:");
      Serial.print(speed);
      Serial.print("x:");
      Serial.print(x);
      Serial.print("y:");
      Serial.print(y);
      Serial.print("z:");
      Serial.println(z);
}
