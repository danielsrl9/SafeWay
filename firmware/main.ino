#include <Adafruit_MPU6050.h>
#include <Adafruit_Sensor.h>
#include <Wire.h>
#include <BluetoothSerial.h> // Biblioteca para comunicação Bluetooth Serial

#define PI 3.14159265358979323846 // Define PI se não estiver disponível

Adafruit_MPU6050 mpu;
BluetoothSerial SerialBT; // Cria um objeto BluetoothSerial

// --- Parâmetros para Detecção de Queda ---
const float LOW_G_THRESHOLD = 3.0;    // Aceleração total em m/s^2 (aprox. 0.3g) - Pode precisar de ajuste!
const float HIGH_G_THRESHOLD = 35.0;  // Aceleração total em m/s^2 (aprox. 3.5g) - Pode precisar de ajuste!
const float ROTATION_THRESHOLD = 400.0; // Velocidade angular total em graus/s - Pode precisar de ajuste!

const unsigned long FALL_WINDOW_MS = 250; // Tempo em ms para a fase de "queda livre"
const unsigned long IMPACT_DEBOUNCE_MS = 5000; // Tempo em ms para evitar múltiplos alertas

bool inFallPhase = false;
unsigned long fallStartTime = 0;
unsigned long lastImpactTime = 0;

// Variáveis para armazenar os eventos do sensor para a função de impressão
sensors_event_t current_a, current_g, current_temp; 


void setup(void) {
  Serial.begin(115200);
 // while (!Serial); // Aguarda a serial para depuração

  // Configuração dos pinos I2C para ESP32 (GPIO 21 SDA, GPIO 22 SCL)
  Wire.begin(21, 22); 

  while (!mpu.begin()) {
    Serial.println("MPU6050 não conectado! Verifique as conexões SDA (GPIO21) e SCL (GPIO22).");
    delay(1000);
  }
  Serial.println("MPU6050 pronto!");

  mpu.setAccelerometerRange(MPU6050_RANGE_16_G); 
  mpu.setGyroRange(MPU6050_RANGE_2000_DEG);
  mpu.setFilterBandwidth(MPU6050_BAND_184_HZ); 

  // --- Configuração do Bluetooth Serial ---
  SerialBT.begin("CapaceteMotoSOS"); // Nome visível do seu dispositivo Bluetooth
  Serial.println("Bluetooth iniciado. Pareie com 'CapaceteMotoSOS'.");
}

void loop() {

  Serial.println("Loop rodando...");

  // Lendo os dados do sensor
  mpu.getEvent(&current_a, &current_g, &current_temp); 

  float totalAcceleration = sqrt(
    current_a.acceleration.x * current_a.acceleration.x +
    current_a.acceleration.y * current_a.acceleration.y +
    current_a.acceleration.z * current_a.acceleration.z
  );

  float totalGyroRotation = sqrt(
    current_g.gyro.x * current_g.gyro.x +
    current_g.gyro.y * current_g.gyro.y +
    current_g.gyro.z * current_g.gyro.z
  );
  totalGyroRotation = totalGyroRotation * 180.0 / PI; 

  unsigned long currentTime = millis();

  // --- Chamar a função para imprimir os dados ---
  printSensorData(totalAcceleration, totalGyroRotation, current_a, current_g);

  // 1. Detecção de Queda (Low-G)
  if (totalAcceleration < LOW_G_THRESHOLD) {
    if (!inFallPhase) { 
      inFallPhase = true;
      fallStartTime = currentTime;
      // Descomente a linha abaixo para depuração adicional
      // Serial.println("INÍCIO DA FASE DE QUEDA (Low-G detectado)"); 
    }
  } else {
    if (inFallPhase && (totalAcceleration > (LOW_G_THRESHOLD + 1.0))) { 
      inFallPhase = false;
      // Descomente a linha abaixo para depuração adicional
      // Serial.println("Fase de queda interrompida"); 
    }
  }

  // 2. Detecção de Impacto (High-G) e Rotação Brusca após a fase de queda
  if (totalAcceleration > HIGH_G_THRESHOLD && totalGyroRotation > ROTATION_THRESHOLD) {
    if (inFallPhase && (currentTime - fallStartTime < FALL_WINDOW_MS)) {
      if (currentTime - lastImpactTime > IMPACT_DEBOUNCE_MS) {
        Serial.println("----------------------------------------");
        Serial.println("     *** ACIDENTE GRAVE DETECTADO! ***");
        Serial.print("     Aceleração de impacto: ");
        Serial.print(totalAcceleration);
        Serial.print(" m/s^2, Rotação: ");
        Serial.print(totalGyroRotation);
        Serial.println(" deg/s");
        Serial.println("----------------------------------------");
        
        // --- Enviar alerta via Bluetooth para o smartphone ---
        if (SerialBT.hasClient()) { 
          String alertMessage = "ACCIDENTE DETECTADO! Aceleracao: " + String(totalAcceleration, 2) + 
                                " m/s^2, Rotacao: " + String(totalGyroRotation, 2) + " deg/s.";
          SerialBT.println(alertMessage); 
          Serial.println("Alerta Bluetooth enviado.");
        } else {
          Serial.println("Nenhum dispositivo Bluetooth conectado para enviar o alerta.");
        }

        inFallPhase = false; 
        lastImpactTime = currentTime; 
      }
    }
  }

  delay(500); 
}

// --- Nova Função para Imprimir Dados no Monitor Serial ---
void printSensorData(float acc_total, float gyro_total, sensors_event_t accel_event, sensors_event_t gyro_event) {
  Serial.print("Accel [X: ");
  Serial.print(accel_event.acceleration.x, 2); // 2 casas decimais
  Serial.print(" Y: ");
  Serial.print(accel_event.acceleration.y, 2);
  Serial.print(" Z: ");
  Serial.print(accel_event.acceleration.z, 2);
  Serial.print("] Total: ");
  Serial.print(acc_total, 2);
  Serial.print(" m/s^2 | Gyro [X: ");
  Serial.print(gyro_event.gyro.x * 180.0 / PI, 2); // Converte para graus/s aqui também
  Serial.print(" Y: ");
  Serial.print(gyro_event.gyro.y * 180.0 / PI, 2);
  Serial.print(" Z: ");
  Serial.print(gyro_event.gyro.z * 180.0 / PI, 2);
  Serial.print("] Total: ");
  Serial.print(gyro_total, 2);
  Serial.println(" deg/s");
}
