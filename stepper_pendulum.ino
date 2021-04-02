#include <Wire.h>
#include <SoftwareSerial.h>
#include <MsTimer2.h>
#include <TimerOne.h>

// k1:傾き　k2:倒れる速度　k3:位置　k4:移動速度
//volatile float k1 = 300.0; volatile  float k2 = 25.0;volatile float k3 = 0.04; volatile float k4 = 1.5; //マイクロステップなし OK
//volatile float k1 = 600.0; volatile  float k2 = 50.0;volatile float k3 = 0.08; volatile float k4 = 3.0; //マイクロステップ1/2 OK
volatile float k1 = 1200.0; volatile  float k2 = 100.0; volatile float k3 = 0.16; volatile float k4 = 6.0; //マイクロステップ1/4 OK
//volatile float k1 = 2400.0; volatile  float k2 = 200.0; volatile float k3 = 0.32; volatile float k4 = 12.0; //マイクロステップ1/8 OK

//30mmホイール1/4ステップは49mmと同じパラメータで倒立する
//volatile int k1 = 1800; volatile  int k2 = 160; volatile int k3 = 0.8; volatile int k4 = 9.0;    //30mmホイール マクロステップ1/4 倒立は安定するが移動してしまう

#define DIR_L   2
#define STEP_L  3
#define DIR_R   4
#define STEP_R  5
#define EN    6

#define DLY   10   // パルス生成のためのTimer1のタイマ割り込み周期 マイクロ秒単位
#define LOOPPERIOD 1 // 制御ループの周期 ミリ秒単位
#define LIMIT 560  // ステッピングモータが脱調しない最大のスピード マイクロステップ1/4=560
#define MOVESPEED 0 //120

SoftwareSerial mySerial(8, 7); // RX, TX

volatile bool outL = false;
volatile bool outR = false;
volatile int countL = 0;
volatile int countR = 0;
volatile int speedL = 0;
volatile int speedR = 0;
volatile long posL = 0;
volatile long posR = 0;
volatile int turnL = 0;
volatile int turnR = 0;
volatile int controlL = 0;
volatile int controlR = 0;


// 加速度・ジャイロセンサーの制御定数
#define MPU6050_ADDR         0x68 // MPU-6050 device address
#define MPU6050_SMPLRT_DIV   0x19 // MPU-6050 register address
#define MPU6050_CONFIG       0x1a
#define MPU6050_GYRO_CONFIG  0x1b
#define MPU6050_ACCEL_CONFIG 0x1c
#define MPU6050_ACCEL_XOUT_H 0x3b
#define MPU6050_ACCEL_XOUT_L 0x3c
#define MPU6050_ACCEL_YOUT_H 0x3d
#define MPU6050_ACCEL_YOUT_L 0x3e
#define MPU6050_ACCEL_ZOUT_H 0x3f
#define MPU6050_ACCEL_ZOUT_L 0x40
#define MPU6050_GYRO_XOUT_H  0x43
#define MPU6050_GYRO_XOUT_L  0x44
#define MPU6050_GYRO_YOUT_H  0x45
#define MPU6050_GYRO_YOUT_L  0x46
#define MPU6050_GYRO_ZOUT_H  0x47
#define MPU6050_GYRO_ZOUT_L  0x48
#define MPU6050_PWR_MGMT_1   0x6b
#define MPU6050_WHO_AM_I     0x75

//倒立振子の制御で使用する変数
volatile long lastuptime;
volatile long starttime;
volatile float dt;
volatile float caribGyroY;
volatile float gyroY;
volatile float degY = 0;
volatile float dpsY = 0;
volatile int cmd;
volatile float lpfY;
volatile float lpfA = 0.999;

// センサーへのコマンド送信
void writeMPU6050(byte reg, byte data) {
  Wire.beginTransmission(MPU6050_ADDR);
  Wire.write(reg);
  Wire.write(data);
  Wire.endTransmission();
}

// センサーからのデータ読み込み
byte readMPU6050(byte reg) {
  Wire.beginTransmission(MPU6050_ADDR);
  Wire.write(reg);
  Wire.endTransmission(false);
  Wire.requestFrom(MPU6050_ADDR, 1, false);
  //readで読み取れるバイト数がなければLED13を消灯
  while (! Wire.available()) {
    digitalWrite(13, LOW);
  }
  byte data =  Wire.read();
  Wire.endTransmission(true);
  return data;
}

void setup() {
  Wire.setClock(400000);
  Serial.begin(115200);
  mySerial.begin(115200);
  Serial.println("*******************RESTARTED********************");
  Serial.println("*****************stepper_pendulum***************");
  Serial.print(k1); Serial.print(","); Serial.print(k2); Serial.print(","); Serial.print(k3); Serial.print(","); Serial.print(k4); Serial.println();

  //フォトインタラプタの初期化
  pinMode(A0, INPUT);
  pinMode(A1, INPUT);

  // モータードライバ制御用ピンの初期化
  pinMode(DIR_L, OUTPUT);
  pinMode(STEP_L, OUTPUT);
  pinMode(DIR_R, OUTPUT);
  pinMode(STEP_R, OUTPUT);
  pinMode(EN, OUTPUT);

  //状態をLED 13にて表示
  pinMode(13, OUTPUT);

  // センサーの初期化
  Wire.begin();
  if (readMPU6050(MPU6050_WHO_AM_I) != 0x68) {
    Serial.println("WHO_AM_I error.");
    while (true) ;
  }
  else {
    Serial.println("WHO_AM_I OK.");
  }
  writeMPU6050(MPU6050_SMPLRT_DIV, 0x07);   // sample rate: 8kHz/(7+1) = 1kHz
  writeMPU6050(MPU6050_CONFIG, 0x00);       // disable DLPF, gyro output rate = 8kHz
  writeMPU6050(MPU6050_GYRO_CONFIG, 0x00);  // gyro range: 0x00⇒±250dps 131LSB、0x08⇒±500dps 65.5LSB、0x10⇒±1000dps 32.8LSB、0x18⇒±2000dps 16.4LSB
  writeMPU6050(MPU6050_ACCEL_CONFIG, 0x00); // accel range: 0x00⇒±2g 16384LSB/g、0x01⇒±4g 8192LSB/g、0x02⇒±8g 4096LSB/g、0x03⇒±16g 2048LSB/g、
  writeMPU6050(MPU6050_PWR_MGMT_1, 0x01);   // disable sleep mode, PLL with X gyro
  Serial.println("MPU6050 Setup OK."); delay(2000);

  //ジャイロのゼロ点調整のために静止時の出力を1000回計測して平均を算出
  caribGyroY = 0;
  for (int i = 0; i < 1000  ; i++)  {
    caribGyroY += (readMPU6050(0x45) << 8) | readMPU6050(0x46);
  }
  caribGyroY /= 1000.0;
  Serial.println("Carib OK.");

  // dt計測用
  lastuptime = micros();

  //　倒立時間計測用
  starttime = micros();

  digitalWrite(EN, LOW);

  Timer1.initialize(DLY); //パルス生成のためのタイマ割込み 引数はマイクロ秒単位
  Timer1.attachInterrupt(pulse);
  MsTimer2::set(LOOPPERIOD, controlloop); //制御のためのタイマ割込み 引数はミリ秒単位
  MsTimer2::start();

  //準備が出来たらLDE 13を点灯
  digitalWrite(13, HIGH);
}

void pulse() {
  countL += controlL;
  if (countL > 10000) {
    // digitalWrite(STEP_L, outL);
    // outL = !outL;
    PORTD ^= 0B00001000;
    countL -= 10000;
  }
  else if (countL < -10000) {
    // digitalWrite(STEP_L, outL);
    // outL = !outL;
    PORTD ^= 0B00001000;
    countL += 10000;
  }
  countR += controlR;
  if (countR > 10000) {
    // digitalWrite(STEP_R, outR);
    // outR = !outR;
    PORTD ^= 0B00100000;
    countR -= 10000;
  }
  else   if (countR < -10000) {
    // digitalWrite(STEP_R, outR);
    // outR = !outR;
    PORTD ^= 0B00100000;
    countR += 10000;
  }
}

void controlloop () {
  // 割り込みハンドラの中でI2C通信(割り込み処理を使用）を許可する
  interrupts();

/*  if (mySerial.available() > 0) {  //シリアルモニタで「改行なし」にしてコマンドを入力
    cmd = mySerial.read();
    mySerial.println(cmd);
    switch ((int)cmd) {
      case 113: //q
        k1 += 100;
        break;
      case 97: //a
        k1 -= 100;
        break;
      case 119: //w
        k2 += 1.0;
        break;
      case 115: //s
        k2 -= 1.0;
        break;
      case 101: //e
        k3 += 0.01;
        break;
      case 100: //d
        k3 -= 0.01;
        break;
      case 114: //r
        k4 += 1.0;
        break;
      case 102: //f
        k4 -= 1.0;
        break;
      case 116: //t
        mySerial.println("Right");
        turnL = 1;
        turnR = -1;
        break;
      case 103: //g
        mySerial.println("Left");
        turnL = -1;
        turnR = 1;
        break;
      case 121: //y
        mySerial.println("Forward");
        turnL = 1;
        turnR = 1;
        break;
      case 104: //h
        mySerial.println("Back");
        turnL = -1;
        turnR = -1;
        break;
      case 98: //b
        mySerial.println("Stop");
        turnL = 0;
        turnR = 0;
        break;
    }
  }
  */
  
  // dt計測
  dt = (micros() - lastuptime) * 0.000001;
  lastuptime = micros();

  // 角速度を取得
  gyroY =  (readMPU6050(MPU6050_GYRO_YOUT_H) << 8) | readMPU6050(MPU6050_GYRO_YOUT_L);
  gyroY -= caribGyroY;
  dpsY = gyroY / 131.0;

  // 角速度を積算して角度を求める
  degY +=  dpsY * dt;

  //ローパスフィルタでドリフトを補正
  lpfY *=  lpfA;
  lpfY +=  (1 - lpfA) * degY;

  // スピードを積算して位置を求める
  posL += speedL;
  posR += speedR;


  // 制御量の計算
  float k1degY; float k2dpsY; float k3posL; float k4speedL;
  speedL += (k1 * (degY - lpfY) + k2 * dpsY + k3 * posL + k4 * speedL) * dt;
  speedR += (k1 * (degY - lpfY) + k2 * dpsY + k3 * posR + k4 * speedR) * dt;
/*  k1degY = k1 * (degY - lpfY);
  k2dpsY = k2 * dpsY + k3;
  k3posL = k3 * posL;
  k4speedL = k4 * speedL;

  speedL += (k1degY + k2dpsY + k3posL + k4speedL) * dt;
  speedR = speedL;
*/
  speedL = constrain(speedL, 0 - LIMIT, LIMIT);
  speedR = constrain(speedR, 0 - LIMIT, LIMIT);

  //if (abs(speedL) == LIMIT) mySerial.print("LIMIT_L!");
  //if (abs(speedR) == LIMIT) mySerial.print("LIMIT_R!");

/*
  Serial.print(k1degY);
  Serial.print(",");
  Serial.print(k2dpsY);
  Serial.print(",");
  Serial.print(k3posL);
  Serial.print(",");
  Serial.println(k4speedL);
*/
  // 前後進、右左折
  if (turnL > 0) {
    controlL = speedL + MOVESPEED;
  }
  else if (turnL < 0) {
    controlL = speedL - MOVESPEED;
  }
  else {
    controlL = speedL;
  }
  if (turnR > 0) {
    controlR = speedR + MOVESPEED;
  }
  else if (turnR < 0) {
    controlR = speedR - MOVESPEED;
  }
  else {
    controlR = speedR;
  }

  // ステッピングモータの回転方向を指定
  if (controlL > 0) {
    digitalWrite(DIR_L, LOW);
  }
  else digitalWrite(DIR_L, HIGH);
  if (controlR > 0) {
    digitalWrite(DIR_R, LOW);
  }
  else digitalWrite(DIR_R, HIGH);

  // 倒れたらモーター停止
  if (20 < abs(degY - lpfY)) {
    speedL = 0;
    speedR = 0;
    digitalWrite(EN, HIGH);
    Serial.println("**** stop ****");

    while (1) {
      // LED13を点滅
      digitalWrite(13, LOW);
      delay(500);
      digitalWrite(13, HIGH);
      delay(500);
    }
  }
}

void loop() {
}
