const int trigPin = 9;
const int echoPin = 10;
const int IN1 = 5; // PWM
const int IN2 = 4; // dirección

float Kp = 6.0;
float Ki = 0.3;
float Kd = 2.5;

float setpoint = 18.0; // altura deseada en cm
float error, previousError = 0;
float integral = 0;
float derivative;
float output;

void setup() {
  Serial.begin(9600);
  pinMode(trigPin, OUTPUT);
  pinMode(echoPin, INPUT);
  pinMode(IN1, OUTPUT);
  pinMode(IN2, OUTPUT);

  // 🧾 Encabezados para el Serial Plotter
  Serial.println("Distancia\tError\tIntegral\tDerivativo\tPWM");
}

float readDistance() {
  int duracion;
  float distance;
  digitalWrite(trigPin, LOW);
  delay(2);
  digitalWrite(trigPin, HIGH);
  delay(10);
  digitalWrite(trigPin, LOW);
  duracion = pulseIn(echoPin, HIGH);
  distance = duracion * 0.034 / 2;
  delay(50);
  return distance;
}

void loop() {
  //Serial.println("Distancia\tError\tIntegral\tDerivativo\tPWM");
  float distance = readDistance();

  error = setpoint - distance;

  // Anti wind-up simple(explicar por que 620 y 10)
  if (integral > 800) {
    integral = 800;
  } else if (integral < 0) {
    integral = 0;
  } else {
    integral += error;
  }

  derivative = error - previousError;
  output = Kp * error + Ki * integral + Kd * derivative;
  previousError = error;

  output = constrain(output, 0, 255); // limitar a rango PWM válido

  // Control del motor
  if (error > 0) {
    analogWrite(IN1, output); // subir
    digitalWrite(IN2, LOW);
  } else {
    analogWrite(IN1, output); // bajar
    digitalWrite(IN2, LOW);
  }

  // 📊 Envío de datos al Plotter Serial
  Serial.print(distance);
  Serial.print("\t");
  Serial.print(error);
  Serial.print("\t");
  Serial.print(integral);
  Serial.print("\t");
  Serial.print(derivative);
  Serial.print("\t");
  Serial.println(output);

  delay(200); // ajustar para suavizar las lecturas
}
