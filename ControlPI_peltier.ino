#include <OneWire.h>
#include <DallasTemperature.h>

#define ONE_WIRE_BUS 2
#define BOTON_PIN 16
#define ACTIVAR_PIN_D1 5
#define ACTIVAR_PIN_D2 4

OneWire oneWire(ONE_WIRE_BUS);
DallasTemperature sensors(&oneWire);

int estado_planta = 0;
unsigned long tiempo_inicial = 0;
int estado_anterior_boton = HIGH;
bool conteo_iniciado = false;

#define KP -397.16 // Constante proporcional (ajústala según sea necesario)
#define KI -0.4433  // Constante integral (ajusta según sea necesario)
#define TEMPERATURA_OBJETIVO -1 // Temperatura deseada en grados Celsius

float error_previo = 0;
float suma_errores = 0;

void setup() {
  Serial.begin(9600);
  sensors.begin();
  pinMode(ACTIVAR_PIN_D1, OUTPUT);
  pinMode(ACTIVAR_PIN_D2, OUTPUT);
  pinMode(BOTON_PIN, INPUT_PULLUP);

}

void loop() {

  int estado_boton = digitalRead(BOTON_PIN);
  sensors.requestTemperatures();

  if (estado_boton == LOW && estado_anterior_boton == HIGH) {
    tiempo_inicial = millis();
    conteo_iniciado = true;
    estado_planta = 0;
  }
  estado_anterior_boton = estado_boton;

  if (conteo_iniciado) {
    unsigned long tiempo_transcurrido = (millis() - tiempo_inicial) / 1000;
    if (tiempo_transcurrido >= 23) {
      estado_planta = 1;
    }
    float temperatura_actual = sensors.getTempCByIndex(0);
    float error = TEMPERATURA_OBJETIVO - temperatura_actual;
    
    suma_errores += error;

    // Control PI
    float potencia = KP * error + KI * suma_errores;

    // Limitar la potencia para evitar valores extremos
    if (potencia > 255) {
      potencia = 255;
    } else if (potencia < 0) {
      potencia = 0;
    }

    // Actualización de variables para la próxima iteración
    error_previo = error;

    Serial.print(tiempo_transcurrido);
    Serial.print(",");//
    Serial.print(temperatura_actual);
    Serial.println();

    
    if (estado_planta == 1) {
      analogWrite(ACTIVAR_PIN_D1, potencia); // Aplicar la potencia a la celda Peltier
      digitalWrite(ACTIVAR_PIN_D2, HIGH);
    } else {
      analogWrite(ACTIVAR_PIN_D1,0); // Aplicar la potencia a la celda Peltier
      digitalWrite(ACTIVAR_PIN_D2, LOW);
    }
  }

  delay(500);
}
