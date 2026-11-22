// CYD Radar - ESP32-2432S028 (Cheap Yellow Display) + SG90 + HC-SR04
// SG90 で HC-SR04 を左右に振り、障害物の方向と距離をレーダー風に表示します。
//
// VS Code + PlatformIO 用 (platformio.ini でライブラリと TFT_eSPI 設定を管理)
//
// ピン割り当て (CYD で実際に空いているのは GPIO 22 / 27 / 35 の3本だけ):
//   SG90 信号 (橙)      -> GPIO27 (CN1)
//   HC-SR04 TRIG        -> GPIO22 (CN1 / P3)
//   HC-SR04 ECHO        -> GPIO35 (P3, 入力専用) ※ 1kΩ/2kΩ の分圧経由必須
//
// 電源の注意:
//   SG90 と HC-SR04 は 5V 駆動です。CYD の 3.3V からは取らないでください。
//   別の 5V/2A 電源 (USB充電器など) から両方へ給電し、GND は CYD と共通に。
//   SG90 の電源線の近くに 470〜1000uF の電解コンデンサを入れると安定します。

#include <Arduino.h>
#include <TFT_eSPI.h>
#include <ESP32Servo.h>

// ---------- ピン ----------
#define SERVO_PIN 27
#define TRIG_PIN  22
#define ECHO_PIN  35   // 入力専用ピン。ECHO の読み取りにちょうどよい

// ---------- レーダーの設定 ----------
#define ANGLE_MIN   15          // 掃引範囲 (度)
#define ANGLE_MAX   165
#define ANGLE_STEP  3           // 1ステップの角度。小さいほど細かいが遅い
#define MAX_RANGE_CM 150        // 表示する最大距離 (cm)
#define SERVO_SETTLE_MS 40      // サーボが動いて落ち着くまでの待ち時間
#define INVERT_DIRECTION 0      // 左右が逆に表示されたら 1 にする

// ---------- 画面 (横向き 320x240) ----------
#define SCREEN_W 320
#define SCREEN_H 240
#define CX 160                  // レーダー中心
#define CY 232
#define R_MAX 148               // 最大距離の半径 (px)

#define COL_BG      TFT_BLACK
#define COL_RING    0x03C0      // グリッドの緑 (暗背景で見える明るさに)
#define COL_SWEEP   TFT_GREEN
#define COL_OBJ     TFT_RED
#define COL_OBJ_HALO 0x7800     // 障害物点のハロー (暗い赤)
#define COL_TEXT    TFT_GREEN

TFT_eSPI tft = TFT_eSPI();
Servo servo;

int distAt[181];                // 角度ごとの最新距離 (cm)。-1 = 障害物なし
int angleNow = ANGLE_MIN;
int angleDir = +1;

float angleToRad(int angle) {
#if INVERT_DIRECTION
  angle = 180 - angle;
#endif
  return angle * DEG_TO_RAD;    // 0度=右、90度=正面、180度=左
}

void polarToXY(int angle, int distCm, int &x, int &y) {
  float r = (float)distCm / MAX_RANGE_CM * R_MAX;
  if (r > R_MAX) r = R_MAX;
  float rad = angleToRad(angle);
  x = CX + (int)(r * cos(rad));
  y = CY - (int)(r * sin(rad));
}

// HC-SR04 で1回測距。障害物がなければ -1 を返す
int measureCm() {
  digitalWrite(TRIG_PIN, LOW);
  delayMicroseconds(2);
  digitalWrite(TRIG_PIN, HIGH);
  delayMicroseconds(10);
  digitalWrite(TRIG_PIN, LOW);

  // タイムアウト 30ms ≒ 約5m。これより遠いものは「なし」扱い
  unsigned long dur = pulseIn(ECHO_PIN, HIGH, 30000UL);
  if (dur == 0) return -1;
  int cm = (int)(dur * 0.0343f / 2.0f);
  if (cm < 2 || cm > MAX_RANGE_CM) return -1;
  return cm;
}

void drawStaticRadar() {
  tft.fillScreen(COL_BG);
  // 同心円 (半円) : 1/3, 2/3, フルレンジ
  for (int i = 1; i <= 3; i++) {
    int r = R_MAX * i / 3;
    for (int a = 0; a <= 180; a += 2) {
      float rad = a * DEG_TO_RAD;
      int x = CX + (int)(r * cos(rad));
      int y = CY - (int)(r * sin(rad));
      tft.drawPixel(x, y, COL_RING);
    }
  }
  // 放射線: 30度ごと
  for (int a = 0; a <= 180; a += 30) {
    float rad = a * DEG_TO_RAD;
    int x = CX + (int)(R_MAX * cos(rad));
    int y = CY - (int)(R_MAX * sin(rad));
    tft.drawLine(CX, CY, x, y, COL_RING);
  }
  tft.drawLine(0, CY, SCREEN_W - 1, CY, COL_RING);
  // 距離ラベル
  tft.setTextColor(COL_RING, COL_BG);
  tft.setTextSize(1);
  tft.drawString("50",  CX - R_MAX / 3 - 8,     CY - R_MAX / 3 - 10);
  tft.drawString("100", CX - 2 * R_MAX / 3 - 10, CY - 2 * R_MAX / 3 - 10);
  tft.drawString("150cm", CX - R_MAX + 2, CY - R_MAX - 2);
}

void drawSweepLine(int angle, uint16_t color) {
  float rad = angleToRad(angle);
  int x = CX + (int)(R_MAX * cos(rad));
  int y = CY - (int)(R_MAX * sin(rad));
  tft.drawLine(CX, CY, x, y, color);
}

void drawObjectDot(int angle, int distCm, uint16_t color, int size) {
  if (distCm < 0) return;
  int x, y;
  polarToXY(angle, distCm, x, y);
  tft.fillCircle(x, y, size, color);
}

void drawHeader(int angle, int distCm) {
  tft.fillRect(0, 0, SCREEN_W, 22, COL_BG);
  tft.setTextColor(COL_TEXT, COL_BG);
  tft.setTextSize(2);
  char buf[40];
  if (distCm >= 0) {
    snprintf(buf, sizeof(buf), "%3d deg  %3d cm", angle, distCm);
  } else {
    snprintf(buf, sizeof(buf), "%3d deg   ---", angle);
  }
  tft.drawString(buf, 6, 3);
}

void setup() {
  Serial.begin(115200);

  pinMode(TRIG_PIN, OUTPUT);
  pinMode(ECHO_PIN, INPUT);   // GPIO35 は内部プルアップなし。分圧回路側で処理
  digitalWrite(TRIG_PIN, LOW);

  for (int i = 0; i <= 180; i++) distAt[i] = -1;

  // ディスプレイ
  tft.init();
  tft.setRotation(1);         // 横向き 320x240
#ifdef TFT_BL
  pinMode(TFT_BL, OUTPUT);
  digitalWrite(TFT_BL, HIGH); // バックライト ON (CYD は GPIO21)
#endif
  drawStaticRadar();

  // サーボ: ESP32Servo は LEDC を使う。50Hz / SG90 向けパルス幅
  ESP32PWM::allocateTimer(3); // TFT 側と競合しにくいタイマを選ぶ
  servo.setPeriodHertz(50);
  servo.attach(SERVO_PIN, 500, 2400);
  servo.write(angleNow);
  delay(500);
}

void loop() {
  // 前回の同じ角度の点を消してから測り直す (残像を1掃引分だけ残す動きになる)
  drawObjectDot(angleNow, distAt[angleNow], COL_BG, 6);

  servo.write(angleNow);
  delay(SERVO_SETTLE_MS);

  int cm = measureCm();
  distAt[angleNow] = cm;

  // 掃引線: 後ろにフェードする残像 (軌跡) を付けて、本物のレーダー風に描く。
  // 消した線が横切った静止画 (リング) は薄れるが、掃引の端で描き直している。
  static const uint16_t TRAIL[] = {0x0140, 0x0240, 0x03C0, 0x0580};  // 尾: 暗→明
  const int trailLen = sizeof(TRAIL) / sizeof(TRAIL[0]);
  int eraseAngle = angleNow - angleDir * (trailLen + 1) * ANGLE_STEP;
  if (eraseAngle >= ANGLE_MIN && eraseAngle <= ANGLE_MAX) {
    drawSweepLine(eraseAngle, COL_BG);
  }
  for (int k = trailLen; k >= 1; k--) {
    int a = angleNow - angleDir * k * ANGLE_STEP;
    if (a < ANGLE_MIN || a > ANGLE_MAX) continue;
    drawSweepLine(a, TRAIL[trailLen - k]);
  }
  drawSweepLine(angleNow, COL_SWEEP);
  // 障害物の点はハロー (暗い赤) 付きで光らせる
  if (cm >= 0) drawObjectDot(angleNow, cm, COL_OBJ_HALO, 5);
  drawObjectDot(angleNow, cm, COL_OBJ, 3);
  drawHeader(angleNow, cm);

  Serial.printf("%d,%d\n", angleNow, cm);  // シリアルでも確認できる (角度,距離cm / -1=なし)

  // 次の角度へ。端で反転し、そのタイミングで静止画を描き直す
  angleNow += angleDir * ANGLE_STEP;
  if (angleNow >= ANGLE_MAX) {
    angleNow = ANGLE_MAX;
    angleDir = -1;
    drawStaticRadar();
  } else if (angleNow <= ANGLE_MIN) {
    angleNow = ANGLE_MIN;
    angleDir = +1;
    drawStaticRadar();
  }
}
