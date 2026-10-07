// FS5103B サーボをシリアルコマンドで制御する (Arduino Uno, 信号線 = D13)
//
// シリアル: 115200bps, 改行区切り
//   <数値>      角度[deg] (0-180) を指定      例: "90"
//   p<数値>     パルス幅[us] (500-2500) を直接指定  例: "p1500"
//   ?           現在のパルス幅を返す
// 応答: "OK <パルス幅us>" / "ERR <理由>"

#include <Servo.h>

// === Steering settings ===
const uint8_t SERVO_PIN = 13;
const int PULSE_MIN_US = 500;   // FS5103B: 500us = 0deg
const int PULSE_MAX_US = 2500;  //          2500us = 180deg
const int INIT_ANGLE = 90;

const int ANGLE_MIN = 40;
const int ANGLE_MAX = 130;
// 40° → 944us, 130° → 1944us
const int LIMIT_MIN_US = PULSE_MIN_US + (long)(PULSE_MAX_US - PULSE_MIN_US) * ANGLE_MIN / 180;
const int LIMIT_MAX_US = PULSE_MIN_US + (long)(PULSE_MAX_US - PULSE_MIN_US) * ANGLE_MAX / 180;


Servo servo;
String line;

void writePulse(int us) {
  us = constrain(us, LIMIT_MIN_US, LIMIT_MAX_US);
  servo.writeMicroseconds(us);
  Serial.print("OK ");
  Serial.println(us);
}

void handle(String cmd) {
  cmd.trim(); // 前後の空白文字を削除
  if (cmd.length() == 0) return;

  if (cmd == "?") {
    Serial.print("OK ");
    Serial.println(servo.readMicroseconds());
    return;
  }

  bool pulseMode = (cmd[0] == 'p' || cmd[0] == 'P');
  String num = pulseMode ? cmd.substring(1) : cmd; // pulseMode=Falseでnum=cmd
  for (unsigned int i = 0; i < num.length(); i++) {
    if (!isDigit(num[i])) {
      Serial.println("ERR invalid number");
      return;
    }
  }
  long v = num.toInt();

  if (pulseMode) {
    writePulse(v);
  } else if (v < ANGLE_MIN || v > ANGLE_MAX) {
    Serial.println("ERR angle out of range (40-130)");
  } else {
    writePulse(map(v, 0, 180, PULSE_MIN_US, PULSE_MAX_US));
  }
}

void setup() {
  Serial.begin(115200);
  
  servo.attach(SERVO_PIN, PULSE_MIN_US, PULSE_MAX_US);
  servo.writeMicroseconds(map(INIT_ANGLE, 0, 180, PULSE_MIN_US, PULSE_MAX_US));
  Serial.println("READY FS5103B on D13");
}

void loop() {
  while (Serial.available()) {
    char c = Serial.read(); // read one character
    if (c == '\n' || c == '\r') {
      handle(line);
      line = "";
    } else if (line.length() < 16) {
      line += c;
    }
  }
}
