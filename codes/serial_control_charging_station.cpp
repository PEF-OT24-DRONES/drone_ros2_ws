int IN1 = 9; // Pin connected to H-bridge IN1
int IN2 = 8; // Pin connected to H-bridge IN2
int forwardLimitSwitch = 10; // Pin connected to the forward limit switch
int backwardLimitSwitch = 11; // Pin connected to the backward limit switch

bool movingForward = false;  // Track if the motor is moving forward
bool movingBackward = false; // Track if the motor is moving backward

void setup() {
  // Configure pins as outputs
  pinMode(IN1, OUTPUT);
  pinMode(IN2, OUTPUT);

  // Configure limit switch pins as inputs
  pinMode(forwardLimitSwitch, INPUT_PULLUP);
  pinMode(backwardLimitSwitch, INPUT_PULLUP);

  // Start serial communication
  Serial.begin(9600);
}

void loop() {
  // Check if data is available in the serial buffer
  if (Serial.available() > 0) {
    // Read the byte that arrives on the serial port
    char command = Serial.read();

    // Process the received command
    if (command == '1') {
      // Start moving forward if the forward limit switch is not pressed
    //  if (digitalRead(forwardLimitSwitch) == LOW) {
        digitalWrite(IN1, HIGH);
        digitalWrite(IN2, LOW);
        movingForward = true;
        movingBackward = false;
    //  }
    } else if (command == '2') {
      // Start moving backward if the backward limit switch is not pressed
      //if (digitalRead(backwardLimitSwitch) == LOW) {
        digitalWrite(IN1, LOW);
        digitalWrite(IN2, HIGH);
        movingForward = false;
        movingBackward = true;
      //}
    } else if (command == '0') {
      // Stop the motor
      digitalWrite(IN1, LOW);
      digitalWrite(IN2, LOW);
      movingForward = false;
      movingBackward = false;
    }
  }

  // Check the limit switches to stop the motor when the correct condition is met
  if (movingForward && digitalRead(backwardLimitSwitch) == LOW) {
    // Stop moving forward when the backward limit switch is pressed
    digitalWrite(IN1, LOW);
    digitalWrite(IN2, LOW);
    movingForward = false;
  }

  if (movingBackward && digitalRead(forwardLimitSwitch) == LOW) {
    // Stop moving backward when the forward limit switch is pressed
    digitalWrite(IN1, LOW);
    digitalWrite(IN2, LOW);
    movingBackward = false;
  }
}
