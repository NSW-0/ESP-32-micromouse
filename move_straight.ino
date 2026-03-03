// L298N -> ESP32
#define ENA 25
#define IN1 26
#define IN2 27

#define ENB 19
#define IN3 17
#define IN4 18

const int PWM_FREQ = 1000;
const int PWM_RES  = 8;

int reverseDuty = 55;   // TUNE THIS (0..255)

void setup() {
  // Motor pins
  pinMode(IN1, OUTPUT); pinMode(IN2, OUTPUT);
  pinMode(IN3, OUTPUT); pinMode(IN4, OUTPUT);

  // PWM
  ledcAttach(ENA, PWM_FREQ, PWM_RES);
  ledcAttach(ENB, PWM_FREQ, PWM_RES);

  // BACKWARD
  digitalWrite(IN1, LOW);  digitalWrite(IN2, HIGH); // Motor A backward
  digitalWrite(IN3, LOW);  digitalWrite(IN4, HIGH); // Motor B backward

  // Speed
  ledcWrite(ENA, reverseDuty);
  ledcWrite(ENB, reverseDuty);
}

void loop() {
  // nothing, just keep reversing
}
