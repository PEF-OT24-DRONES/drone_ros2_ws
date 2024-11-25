int IN1 = 9; // Pin connected to H-bridge IN1 (PWM)
int IN2 = 8; // Pin connected to H-bridge IN2 (PWM)
int forwardLimitSwitch = 11; // Pin connected to the forward limit switch
int backwardLimitSwitch = 10; // Pin connected to the backward limit switch
int JoyStick_X = A0; // X-axis-signal
int JoyStick_Y = A1; // Y-axis-signal

bool movingForward = false;  // Track if the motor is moving forward
bool movingBackward = false; // Track if the motor is moving backward
bool commandActive = false;  // Track if a command is currently active
char currentCommand = '0';   // Track the current active command

int motorSpeed = 100;        // Default motor speed (0-255, where 255 is max speed)

void setup() {
  // Configure pins as outputs
  pinMode(IN1, OUTPUT);
  pinMode(IN2, OUTPUT);
  pinMode(JoyStick_X, INPUT);
  pinMode(JoyStick_Y, INPUT);

  // Configure limit switch pins as inputs
  pinMode(forwardLimitSwitch, INPUT_PULLUP);
  pinMode(backwardLimitSwitch, INPUT_PULLUP);

  // Start serial communication
  Serial.begin(9600);
}

void loop() {
  // Read the current values of the joystick
  int xValue = analogRead(JoyStick_X);
  int yValue = analogRead(JoyStick_Y);

  // Check if data is available in the serial buffer
  if (Serial.available() > 0) {
    char command = Serial.read();
    if (command == '1' || command == '2' || command == '0') {
      currentCommand = command;
      commandActive = (command != '0');
    } else if (command == '+') { // Increase speed
      motorSpeed = min(255, motorSpeed + 10); // Increment speed, max 255
      Serial.print("Increased speed to: ");
      Serial.println(motorSpeed);
    } else if (command == '-') { // Decrease speed
      motorSpeed = max(0, motorSpeed - 10); // Decrement speed, min 0
      Serial.print("Decreased speed to: ");
      Serial.println(motorSpeed);
    }
  }

  // Handle joystick override
  if (yValue > 550) { // Joystick forward
    Serial.println("Joystick: Motor Enfrente");
    stopMotor();
    moveForward();
  } else if (yValue < 480) { // Joystick backward
    Serial.println("Joystick: Motor Atras");
    stopMotor();
    moveBackward();
  } else if (!commandActive) { // No joystick input and no active command
    stopMotor();
  }

  // Handle serial commands
  if (commandActive) {
    if (currentCommand == '1') { // Open (move forward)
      Serial.println("Serial: Motor Enfrente (Command Active)");
      moveForward();
      if (digitalRead(backwardLimitSwitch) == LOW) {
        Serial.println("Backward limit switch pressed. Stopping motor.");
        stopMotor();
        commandActive = false;
      }
    } else if (currentCommand == '2') { // Close (move backward)
      Serial.println("Serial: Motor Atras (Command Active)");
      moveBackward();
      if (digitalRead(forwardLimitSwitch) == LOW) {
        Serial.println("Forward limit switch pressed. Stopping motor.");
        stopMotor();
        commandActive = false;
      }
    } else if (currentCommand == '0') { // Stop command
      Serial.println("Serial: Motor Parado");
      stopMotor();
      commandActive = false;
    }
  }

  delay(100); // Small delay to stabilize readings
}

// Helper functions
void moveForward() {
  analogWrite(IN1, 0); // Set speed for IN1
  analogWrite(IN2, motorSpeed);          // Stop IN2
  movingForward = true;
  movingBackward = false;
}

void moveBackward() {
  analogWrite(IN1, motorSpeed);          // Stop IN1
  analogWrite(IN2, 0); // Set speed for IN2
  movingForward = false;
  movingBackward = true;
}

void stopMotor() {
  analogWrite(IN1, 0); // Stop IN1
  analogWrite(IN2, 0); // Stop IN2
  movingForward = false;
  movingBackward = false;
}
