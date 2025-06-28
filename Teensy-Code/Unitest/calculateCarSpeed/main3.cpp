#define DIR1 4
#define PWM1 5
#define DIR2 7
#define PWM2 6

#define encoder1Pin 2
#define encoder2Pin 3

volatile long encoder1Count = 0;
volatile long encoder2Count = 0;

long previousTime = 0;
float ePrevious = 0;
float eIntegral = 0;

void setup() {
    Serial.begin(9600);

    pinMode(DIR1, OUTPUT);
    pinMode(PWM1, OUTPUT);
    pinMode(DIR2, OUTPUT);
    pinMode(PWM2, OUTPUT);
    pinMode(encoder1Pin, INPUT);
    pinMode(encoder2Pin, INPUT);

    attachInterrupt(digitalPinToInterrupt(encoder1Pin), handleEncoder1, RISING);
    attachInterrupt(digitalPinToInterrupt(encoder2Pin), handleEncoder2, RISING);
}

void loop() {
    int target = encoder1Count;

    //move motor 1
    digitalWrite(DIR1, 1);
    analogWrite(PWM1, 50);

    float kp = 2.0;
    float kd = 0.0;
    float ki = 0.0;
    float u = pidController(target, kp, kd, ki);

    moveMotor(DIR2, PWM2, u);

    Serial.print(encoder1Count);
    Serial.print(", ");
    Serial.println(encoder2Count); // Use println instead of print for new line
}

void handleEncoder1() {
    encoder1Count++;
}

void handleEncoder2() {
    encoder2Count++;
}

void moveMotor(int dirPin, int pwmPin, float u) {
    float speed = fabs(u);
    if (speed > 255) {
        speed = 255;
    }
    if (encoder2Count > encoder1Count) {
        speed = 0; // stop the speed
    }
    int direction = 0;
    
    digitalWrite(dirPin, direction);
    analogWrite(pwmPin, speed);
}

float pidController(int target, float kp, float kd, float ki) {
    long currentTime = micros();
    float deltaT = ((float)(currentTime - previousTime)) / 1.0e6;

    int e = encoderCount - target;
    float eDerivative = (e - ePrevious) / deltaT;
    eIntegral = eIntegral + e * deltaT;

    float u = (kp * e) + (kd * eDerivative) + (ki * eIntegral);

    previousTime = currentTime;
    ePrevious = e;

    return u;
}