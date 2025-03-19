#include <Wire.h>
#include <Adafruit_PWMServoDriver.h>

// Initialize the servo driver
Adafruit_PWMServoDriver pwm = Adafruit_PWMServoDriver();

// Servo motor specifications
#define SERVOMIN 150  // Minimum pulse length
#define SERVOMAX 600  // Maximum pulse length

// Servo channel assignments
#define BASE_SERVO 0     // 360-degree servo (channel 0)
#define ARM_SERVO_1 1    // 180-degree servo (channel 1)
#define ARM_SERVO_2 2    // 180-degree servo (channel 2)

// Rotation speed configuration
#define STEP_DELAY 50 // Delay between each small movement step in milliseconds
#define STEP_SIZE 1   // Angle step size (degrees)

void setup() {
  Serial.begin(9600);
  Serial.println("3-DOF Robotic Arm Initialization");

  pwm.begin();
  pwm.setPWMFreq(50); // Set the PWM frequency to 50 Hz

  // Ensure the base servo is stationary at its initial position (e.g., 90 degrees)
  moveServoSlow(BASE_SERVO, 90, 0); // Initial position (no delay)
}

void loop() {
  // Example movements for the other arm servos
  
  moveServoSlow(ARM_SERVO_1, 45, STEP_DELAY); // Move arm 1 to 45 degrees slowly
  delay(1000);
  moveServoSlow(ARM_SERVO_1, 135, STEP_DELAY); // Move arm 1 to 135 degrees slowly
  delay(1000);

  moveServoSlow(ARM_SERVO_2, 30, STEP_DELAY); // Move arm 2 to 30 degrees slowly
  delay(1000);
  moveServoSlow(ARM_SERVO_2, 120, STEP_DELAY); // Move arm 2 to 120 degrees slowly
  delay(1000);
}

// Function to move a servo to a specific angle with slow rotation
void moveServoSlow(int servoChannel, int targetAngle, int delayTime) {
  static int currentAngle[16] = {0}; // Track the current position of each servo (initialize to 0 degrees)

  if (targetAngle < 0 || targetAngle > 180) {
    Serial.print("Invalid angle: ");
    Serial.println(targetAngle);
    return;
  }

  // Gradually move the servo to the target angle
  int step = (targetAngle > currentAngle[servoChannel]) ? STEP_SIZE : -STEP_SIZE;
  while (currentAngle[servoChannel] != targetAngle) {
    currentAngle[servoChannel] += step;

    // Prevent overshooting
    if ((step > 0 && currentAngle[servoChannel] > targetAngle) ||
        (step < 0 && currentAngle[servoChannel] < targetAngle)) {
      currentAngle[servoChannel] = targetAngle;
    }

    int pulseLength = mapAngleToPulse(currentAngle[servoChannel]);
    pwm.setPWM(servoChannel, 0, pulseLength);

    Serial.print("Servo ");
    Serial.print(servoChannel);
    Serial.print(" moved to ");
    Serial.print(currentAngle[servoChannel]);
    Serial.println(" degrees.");

    delay(delayTime);
  }
}

// Function to map angles to pulse lengths
int mapAngleToPulse(int angle) {
  return map(angle, 0, 180, SERVOMIN, SERVOMAX);
}
