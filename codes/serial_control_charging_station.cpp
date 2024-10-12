int IN1 = 8; // Pin conectado a H-bridge IN1
int IN2 = 9; // Pin conectado a H-bridge IN2

void setup() {
  // Configuración de los pines como salidas
  pinMode(IN1, OUTPUT);
  pinMode(IN2, OUTPUT);
  
  // Inicia la comunicación serial
  Serial.begin(9600);
}

void loop() {
  // Verifica si hay datos disponibles en el buffer serial
  if (Serial.available() > 0) {
    // Lee el byte que llega por el puerto serial
    char command = Serial.read();
    
    // Procesa el comando recibido
    if (command == '1') {
      // Mueve el motor hacia adelante
      digitalWrite(IN1, HIGH);
      digitalWrite(IN2, LOW);
    } else if (command == '2') {
      // Mueve el motor hacia atrás (reversa)
      digitalWrite(IN1, LOW);
      digitalWrite(IN2, HIGH);
    } else if (command == '0') {
      // Detiene el motor
      digitalWrite(IN1, LOW);
      digitalWrite(IN2, LOW);
    }
  }
}