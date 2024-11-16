// #define IR_SENSOR_RIGHT 10  // Front IR sensor
#define IR_SENSOR_LEFT 9   // Back IR sensor
#define MOTOR_SPEED 50  // Motor speed constant (0-255)

// Motor pins
const int enableRightMotor = 5;
const int rightMotorPin1 = 10;
const int rightMotorPin2 = 11;
const int enableLeftMotor = 3;
const int leftMotorPin1 = 12;
const int leftMotorPin2 = 13;

void setup() {
  // Initialize IR sensor pins
  // pinMode(IR_SENSOR_RIGHT, INPUT);
  pinMode(IR_SENSOR_LEFT, INPUT);
  
  // Initialize motor control pins
  pinMode(enableRightMotor, OUTPUT);
  pinMode(rightMotorPin1, OUTPUT);
  pinMode(rightMotorPin2, OUTPUT);
  pinMode(enableLeftMotor, OUTPUT);
  pinMode(leftMotorPin1, OUTPUT);
  pinMode(leftMotorPin2, OUTPUT);
  
  Serial.begin(9600); // For debugging
}

void loop() {
  // Read the state of the IR sensors
  // int frontObstacle = digitalRead(IR_SENSOR_RIGHT); // Front sensor
  int backObstacle = digitalRead(IR_SENSOR_LEFT); 
  // Serial.println(frontObstacle) ;
   Serial.println(backObstacle) ; // Back sensor

  if ( backObstacle == HIGH) {
    // Obstacle detected in the front, move backward
    moveBackward();
  } else if (backObstacle == HIGH) {
    // Obstacle detected in the back, move forward
    moveForward();
  } else {
    // No obstacle or obstacles in both directions, stop
    stopMovement();
  }

  delay(100); // Small delay to avoid excessive sensor polling
}

void moveForward() {
  // Set motors for forward movement
  digitalWrite(rightMotorPin1, HIGH);
  digitalWrite(rightMotorPin2, LOW);
  digitalWrite(leftMotorPin1, HIGH);
  digitalWrite(leftMotorPin2, LOW);

  // Set motor speed
  analogWrite(enableRightMotor, MOTOR_SPEED);
  analogWrite(enableLeftMotor, MOTOR_SPEED);

  Serial.println("Moving Forward");
}

void moveBackward() {
  // Set motors for backward movement
  digitalWrite(rightMotorPin1, LOW);
  digitalWrite(rightMotorPin2, HIGH);
  digitalWrite(leftMotorPin1, LOW);
  digitalWrite(leftMotorPin2, HIGH);

  // Set motor speed
  analogWrite(enableRightMotor, MOTOR_SPEED);
  analogWrite(enableLeftMotor, MOTOR_SPEED);

  Serial.println("Moving Backward");
}

void stopMovement() {
  // Stop both motors
  digitalWrite(rightMotorPin1, LOW);
  digitalWrite(rightMotorPin2, LOW);
  digitalWrite(leftMotorPin1, LOW);
  digitalWrite(leftMotorPin2, LOW);

  // Turn off motor speed to stop motors
  analogWrite(enableRightMotor, 0);
  analogWrite(enableLeftMotor, 0);

  Serial.println("Stopping");
}
