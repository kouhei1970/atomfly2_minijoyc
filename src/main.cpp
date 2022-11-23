#include <M5StickCPlus.h>
#include <MadgwickAHRS.h>
#include <WiFi.h>
#include <esp_now.h>

Madgwick Ahrs;
esp_now_peer_info_t slave;

float accX = 0.0F;
float accY = 0.0F;
float accZ = 0.0F;
float gyroX = 0.0F;
float gyroY = 0.0F;
float gyroZ = 0.0F;
float gyroX_bias=0.0;
float gyroY_bias=0.0;
float gyroZ_bias=0.0;
float Ax, Ay, Az;
float Wp, Wq, Wr;
float Phi, Theta, Psi;
float Phi_bias =0.0;
float Theta_bias = 0.0;
float Psi_bias =0.0;
short xstick_bias = 0.0;
short ystick_bias = 0.0;
short xstick=0;
short ystick=0;
short button=0;
unsigned long stime,etime,dtime;
byte axp_cnt=0;



void rc_init(void);
void OnDataSent(const uint8_t *mac_addr, esp_now_send_status_t status);
void data_send(void);
void show_battery_info();

void rc_init(void)
{
  // ESP-NOW初期化
  WiFi.mode(WIFI_STA);
  WiFi.disconnect();
  if (esp_now_init() == ESP_OK) {
    Serial.println("ESPNow Init Success");
  } else {
    Serial.println("ESPNow Init Failed");
    ESP.restart();
  }
#if 1
  // マルチキャスト用Slave登録
  memset(&slave, 0, sizeof(slave));
  for (int i = 0; i < 6; ++i) {
    slave.peer_addr[i] = (uint8_t)0xff;
  }
  
  esp_err_t addStatus = esp_now_add_peer(&slave);
  if (addStatus == ESP_OK) {
    // Pair success
    Serial.println("Pair success");
  }
  // ESP-NOWコールバック登録
  esp_now_register_send_cb(OnDataSent);
  //esp_now_register_recv_cb(OnDataRecv);

    // Ps3.attach(notify);
    // Ps3.attachOnConnect(onConnect);
    // Ps3.begin(BTID);
    Serial.println("Ready.");
  #endif
}

// 送信コールバック
void OnDataSent(const uint8_t *mac_addr, esp_now_send_status_t status) {
  //char macStr[18];
  //snprintf(macStr, sizeof(macStr), "%02X:%02X:%02X:%02X:%02X:%02X",
  //         mac_addr[0], mac_addr[1], mac_addr[2], mac_addr[3], mac_addr[4], mac_addr[5]);
  //Serial.print("Last Packet Sent to: ");
  //Serial.println(macStr);
  //Serial.print("Last Packet Send Status: ");
  //Serial.println(status == ESP_NOW_SEND_SUCCESS ? "Delivery Success" : "Delivery Fail");
}

void data_send(void)
{
  //uint8_t data[13] = {72, 101, 108, 108, 111, 32, 69, 83, 80, 45, 78, 79, 87};
  //esp_err_t result = esp_now_send(slave.peer_addr, data, sizeof(data));
  //Serial.print("Send Status: ");
  //if (result == ESP_OK) {
  //  Serial.println("Success");
  //} else if (result == ESP_ERR_ESPNOW_NOT_INIT) {
  //  Serial.println("ESPNOW not Init.");
  //} else if (result == ESP_ERR_ESPNOW_ARG) {
  //  Serial.println("Invalid Argument");
  //} else if (result == ESP_ERR_ESPNOW_INTERNAL) {
  //  Serial.println("Internal Error");
  //} else if (result == ESP_ERR_ESPNOW_NO_MEM) {
  //  Serial.println("ESP_ERR_ESPNOW_NO_MEM");
  //} else if (result == ESP_ERR_ESPNOW_NOT_FOUND) {
  //  Serial.println("Peer not found.");
  //} else {
  //  Serial.println("Not sure what happened");
  //}  
}

void setup() {

  M5.begin();
  Wire.begin(0, 26);
  Wire1.begin(21, 22);
  M5.IMU.Init();
  Ahrs.begin(100.0);
  rc_init();
  M5.Lcd.fillScreen(BLUE);
  M5.Lcd.setTextSize(2);
  M5.Lcd.setTextColor(WHITE, BLUE);
  M5.Lcd.println("Gyro calibration...");           

  for(int i=0;i<200;i++)
  {
    M5.IMU.getGyroData(&gyroX, &gyroY, &gyroZ);
    gyroX_bias += gyroX;
    gyroY_bias += gyroY;
    gyroZ_bias += gyroZ;
    delay(10);
  }
  gyroX_bias /= 200;
  gyroY_bias /= 200;
  gyroZ_bias /= 200;

  //Display init
  M5.Lcd.fillScreen(RED);       // 画面全体の塗りつぶし
  //M5.Lcd.setCursor(9, 10);      // カーソル位置の指定
  M5.Lcd.setTextFont(1);        // フォントの指定
  M5.Lcd.setTextSize(2);        // フォントサイズを指定（倍数）
  M5.Lcd.setTextColor(WHITE, RED);
  //M5.Lcd.println("AtomFly2.0");           
  //for (uint8_t i=0;i<50;i++)show_battery_info();

  byte error, address;
  int nDevices;

  Serial.println("Scanning... Wire");

  nDevices = 0;
  for (address = 1; address < 127; address++ )
  {
    Wire.beginTransmission(address);
    error = Wire.endTransmission();

    if (error == 0)
    {
      Serial.print("I2C device found at address 0x");
      if (address < 16)
        Serial.print("0");
      Serial.print(address, HEX);
      Serial.println("  !");

      nDevices++;
    }
    else if (error == 4)
    {
      Serial.print("Unknown error at address 0x");
      if (address < 16)
        Serial.print("0");
      Serial.println(address, HEX);
    }
  }
  if (nDevices == 0)
    Serial.println("No I2C devices found\n");
  else
    Serial.println("done\n");

////////////////////////////////////////////////////////
  Serial.println("Scanning... Wire1");

  nDevices = 0;
  for (address = 1; address < 127; address++ )
  {
    Wire1.beginTransmission(address);
    error = Wire1.endTransmission();

    if (error == 0)
    {
      Serial.print("I2C device found at address 0x");
      if (address < 16)
        Serial.print("0");
      Serial.print(address, HEX);
      Serial.println("  !");

      nDevices++;
    }
    else if (error == 4)
    {
      Serial.print("Unknown error at address 0x");
      if (address < 16)
        Serial.print("0");
      Serial.println(address, HEX);
    }
  }
  if (nDevices == 0)
    Serial.println("No I2C devices found\n");
  else
    Serial.println("done\n");

}

char data[140];
uint8_t senddata[11];
uint8_t disp_counter=0;

void loop() {
  byte rx_data[5];
  short _xstick,_ystick;

  stime = micros();

  M5.update();
  
  Wire.beginTransmission(0x54);//I2Cスレーブ「Arduino Uno」のデータ送信開始
  Wire.write(0x10);//x軸指定
  Wire.endTransmission();//I2Cスレーブ「Arduino Uno」のデータ送信終了

  Wire.requestFrom(0x54, 2);  //Request 3 bytes from the slave device.  
  if (Wire.available()) { //If data is received.
    rx_data[0] = Wire.read();
    rx_data[1] = Wire.read();
    _xstick=rx_data[1]*256+rx_data[0];
  }

  Wire.beginTransmission(0x54);//I2Cスレーブ「Arduino Uno」のデータ送信開始
  Wire.write(0x12);//y軸指定
  Wire.endTransmission();//I2Cスレーブ「Arduino Uno」のデータ送信終了

  Wire.requestFrom(0x54, 2);  //Request 3 bytes from the slave device.  
  if (Wire.available()) { //If data is received.
    rx_data[2] = Wire.read();
    rx_data[3] = Wire.read();
    _ystick=rx_data[3]*256+rx_data[2];
    
  }

  Wire.beginTransmission(0x54);//I2Cスレーブ「Arduino Uno」のデータ送信開始
  Wire.write(0x30);//ボタン指定
  Wire.endTransmission();//I2Cスレーブ「Arduino Uno」のデータ送信終了

  Wire.requestFrom(0x54, 1);  //Request 3 bytes from the slave device.  
  if (Wire.available()) { //If data is received.
    rx_data[4] = Wire.read();
    button=rx_data[4];
  }


////////////////////////////////////////////////////////////

  M5.IMU.getAccelData(&accX, &accY, &accZ);
  M5.IMU.getGyroData(&gyroX, &gyroY, &gyroZ);
  gyroX = gyroX - gyroX_bias;
  gyroY = gyroY - gyroY_bias;
  gyroZ = gyroZ - gyroZ_bias;

  Ahrs.updateIMU(gyroX, gyroY, gyroZ, accX, accY, accZ);

  Ax = accX;
  Ay = accY;
  Az = accZ;
  Wp = gyroX*DEG_TO_RAD;
  Wq = gyroY*DEG_TO_RAD;
  Wr = gyroZ*DEG_TO_RAD;

  float _theta = Ahrs.getRoll();//*DEG_TO_RAD;
  float _phi = Ahrs.getPitch();//*DEG_TO_RAD;
  float _psi = Ahrs.getYaw();//*DEG_TO_RAD;

  if(button==0)
  {
    Phi_bias = _phi;
    Theta_bias = _theta;
    Psi_bias = _psi;
    xstick_bias = _xstick;
    ystick_bias = _ystick;
  }

  xstick = _xstick - xstick_bias;
  ystick = _ystick - ystick_bias;
  Phi = _phi - Phi_bias;
  Theta = _theta - Theta_bias;
  Psi = _psi - Psi_bias;

  senddata[0]=(unsigned short)(xstick)/256;
  senddata[1]=(unsigned short)(xstick)%256;

  senddata[2]=(unsigned short)(ystick)/256;
  senddata[3]=(unsigned short)(ystick)%256;

  senddata[4]=(unsigned short)(Phi*1000)/256;
  senddata[5]=(unsigned short)(Phi*1000)%256;

  senddata[6]=(unsigned short)(Theta*1000)/256;
  senddata[7]=(unsigned short)(Theta*1000)%256;

  senddata[8]=(short)(Psi*1000)/256;
  senddata[9]=(short)(Psi*1000)%256;

  senddata[10]=button;

  esp_err_t result = esp_now_send(slave.peer_addr, senddata, sizeof(senddata));

  
  //sprintf(data, "x:%4d y:%4d Phi: %7.3f Theta: %7.3f Psi: %7.3f Btn: %2d Delta:%6d\n", 
  //  xstick, ystick, Phi, Theta, Psi, button, dtime);
  //Serial.print(data);

  if(M5.BtnA.isPressed())
  {
  }

  float vbat = M5.Axp.GetBatVoltage();
  int8_t bat_charge_p = int8_t((vbat - 3.0) / 1.2 * 100);
  
  M5.Lcd.setCursor(2, 5+disp_counter*17);
  switch (disp_counter)
  {
    case 0:
      M5.Lcd.printf("AtomFly2.0");
      break;
    case 1:
      M5.Lcd.printf("X:%4d",xstick);
      break;
    case 2:
      M5.Lcd.printf("Y:%4d",ystick);
      break;
    case 3:
      M5.Lcd.printf("Phi:%5.1f",Phi);
      break;
    case 4:
      M5.Lcd.printf("Tht:%5.1f",Theta);
      break;
    case 5:
      M5.Lcd.printf("Psi:%5.1f",Psi);
      break;
    case 6:
      M5.Lcd.printf("FPS:%5.1f",1000000.0/dtime);
      break;
    case 7:
      M5.Lcd.printf("Vlt:%4.1fV", vbat);
      break;
    case 8:
      M5.Lcd.printf("Chg:%3d%%", bat_charge_p);
      break;        
  }
  disp_counter++;
  if(disp_counter==9)disp_counter=0;

  if( M5.Axp.GetBtnPress() == 2 ){
    // 電源ボタンクリック
    //M5.Lcd.println("AtomFly2.0"); 
    esp_restart();
  } 

  etime = micros();
  dtime = etime - stime;
  //if((10000-dtime)>0)delay((10000-dtime)/1000);

}


void show_battery_info(){
  // バッテリー電圧表示
  double vbat = 0.0;
  int8_t bat_charge_p = 0;

  vbat = M5.Axp.GetBatVoltage();
  M5.Lcd.setCursor(5, 100);
  //M5.Lcd.setTextSize(1);
  M5.Lcd.printf("Volt:\n %8.2fV", vbat);

  // バッテリー残量表示
  bat_charge_p = int8_t((vbat - 3.0) / 1.2 * 100);
  M5.Lcd.setCursor(5, 140);
  M5.Lcd.printf("Charge:\n %8d%%", bat_charge_p);
}
