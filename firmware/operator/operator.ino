#include <Servo.h>

// === Steering settings ===
const uint8_t SERVO_PIN = 13;
const int PULSE_MIN_US = 500;   // FS5103B: 500us = 0deg
const int PULSE_MAX_US = 2500;  //          2500us = 180deg
const int INIT_ANGLE = 85;

const int ANGLE_MIN = 40;
const int ANGLE_MAX = 130;
// 40° → 944us, 130° → 1944us
const int LIMIT_MIN_US = PULSE_MIN_US + (long)(PULSE_MAX_US - PULSE_MIN_US) * ANGLE_MIN / 180;
const int LIMIT_MAX_US = PULSE_MIN_US + (long)(PULSE_MAX_US - PULSE_MIN_US) * ANGLE_MAX / 180;

// === Throttle Settings ===
const uint8_t CTRL_FULL_PIN = 11;
const uint8_t CTRL_INA_PIN = 10;
const uint8_t CTRL_INB_PIN = 9;
const uint8_t CTRL_BRK_PIN = 8;
const uint8_t CTRL_PWM_PIN = 7;
const uint8_t CTRL_PSB_PIN = 6;
const uint8_t CTRL_MODE_PIN = 5;

const int THROTTLE_LIMIT_MIN = -100;
const int THROTTLE_LIMIT_MAX = 100;


Servo servo;
String line;

void writePulse(int us) {
  us = constrain(us, LIMIT_MIN_US, LIMIT_MAX_US);
  servo.writeMicroseconds(us);
  Serial.print("[ INFO ][ STEERING ] OK ");
  Serial.println(us);
}

void handle(int v) {
    if (v < ANGLE_MIN || v > ANGLE_MAX) {
        Serial.println("[ ERR ][ STEERING ] angle out of range (40-130).");
    } else {
        writePulse(map(v, 0, 180, PULSE_MIN_US, PULSE_MAX_US));
    }
}

void forward(int v){
    digitalWrite(CTRL_INA_PIN, 0);
    analogWrite(CTRL_INB_PIN, v);
}

void backward(int v){
    analogWrite(CTRL_INA_PIN, v);
    digitalWrite(CTRL_INB_PIN, 0);
}

void stop(int v){ // feature
    digitalWrite(CTRL_BRK_PIN, HIGH);
    //digitalWrite(DRIVE_M1_PIN, 0);
    //digitalWrite(DRIVE_M2_PIN, 0);
}

void accel(int v){
    if (v < THROTTLE_LIMIT_MIN || v > THROTTLE_LIMIT_MAX){
        Serial.println("[ ERR ][ THROTTLE ] throttle out of range (-100-100).");
    } else {
        if (v >= 0){
            forward(map(v, 0, 100, 0, 255));
        } else {
            backward(map(-v, 0, 100, 0, 255));
        }
        Serial.print("[ INFO ][ THROTTLE ] OK ");
        Serial.println(v);
    }
}

int controller(String cmd){
    cmd.trim();
    if (cmd.length() == 0) return;

    int cmds[2] = {0};

    for (unsigned int i = 0; i < cmd.length(); i++){
        if (!isDigit(cmd[i]) and (cmd[i] != ' ') and (cmd[i] != '-')){ // '-' は後退
            Serial.println("[ ERR ][ CONTROLLER ] Command must integer.");
            Serial.println(cmd);
            return;
        }
    }
    
    unsigned int split_idx;
    bool has_split = false;
    for (unsigned int i = 0; i < cmd.length(); i++){
        if (cmd[i] == ' '){ // ""は文字列 ''は文字
            split_idx = i;
            has_split = true;
        }
    }
    if (has_split){
        long steering_cmd = cmd.substring(0, split_idx).toInt();
        long throttle_cmd = cmd.substring(split_idx).toInt();

        handle(steering_cmd);
        accel(throttle_cmd);
    } else {
        Serial.println("[ ERR ][ CONTROLLER ] Invalid Command.");
    }
}

void setup() {
  Serial.begin(115200);
  
  servo.attach(SERVO_PIN, PULSE_MIN_US, PULSE_MAX_US);
  servo.writeMicroseconds(map(INIT_ANGLE, 0, 180, PULSE_MIN_US, PULSE_MAX_US));

  pinMode(CTRL_FULL_PIN, OUTPUT);
  pinMode(CTRL_INA_PIN, OUTPUT);
  pinMode(CTRL_INB_PIN, OUTPUT);
  pinMode(CTRL_BRK_PIN, OUTPUT);
  pinMode(CTRL_PWM_PIN, OUTPUT);
  pinMode(CTRL_PSB_PIN, OUTPUT);
  pinMode(CTRL_MODE_PIN, OUTPUT);

  digitalWrite(CTRL_FULL_PIN, LOW);
  digitalWrite(CTRL_INA_PIN, LOW);
  digitalWrite(CTRL_INB_PIN, LOW);
  digitalWrite(CTRL_BRK_PIN, LOW);
  digitalWrite(CTRL_PWM_PIN, LOW);
  digitalWrite(CTRL_PWM_PIN, HIGH);
  digitalWrite(CTRL_PSB_PIN, HIGH);
  digitalWrite(CTRL_MODE_PIN, LOW);
  
  Serial.println("[ READY ] Operator start.");
}

void loop() {
  while (Serial.available()) {
    char c = Serial.read(); // read one character
    if (c == '\n' || c == '\r') {
      controller(line);
      line = "";
    } else if (line.length() < 16) {
      line += c;
    }
  }
}
