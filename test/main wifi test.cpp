 #include <Arduino.h>
#include <AccelStepper.h>
#include <ESP32Servo.h>
#include <WiFi.h>
#include <WiFiClient.h>
#include <ESPAsyncWebSrv.h>


#define WIFI_SSID "Wokwi-GUEST"
#define WIFI_PASSWORD ""
// Defining the WiFi channel speeds up the connection:
#define WIFI_CHANNEL 6
AsyncWebServer server(80);

// Structure de configuration
struct Parametre {
  const char* nom;
  int valeur;
};

Parametre config[] = {
  {"bouchonsParBoiteB", 1},
};

const int NB_PARAMS = sizeof(config) / sizeof(config[0]);


// === Définition des broches ===
#define X_STEP_PIN         2
#define X_DIR_PIN          4
#define Y_STEP_PIN         16
#define Y_DIR_PIN          17
#define Z_STEP_PIN         18
#define Z_DIR_PIN          19
#define LED_PIN            5
#define ENDSTOP_X_PIN      32
#define ENDSTOP_Y_PIN      33
#define ENDSTOP_Z_PIN      26
#define HOMING_BUTTON      13
#define START_BUTTON       12
#define SERVO_PIN          27
#define SERVO_BUTTON_PIN   14
#define BOUCHON_BUTTON     34
#define LED_BOUCHON        21
#define EMERGENCY_STOP_PIN 25
#define BUZZER_PIN         22

// === Moteurs ===
AccelStepper stepperX(AccelStepper::DRIVER, X_STEP_PIN, X_DIR_PIN);
AccelStepper stepperY(AccelStepper::DRIVER, Y_STEP_PIN, Y_DIR_PIN);
AccelStepper stepperZ(AccelStepper::DRIVER, Z_STEP_PIN, Z_DIR_PIN);

// === Servo ===
Servo myServo;
bool servoState = false;

// === Paramètres de matrice ===
int lignesA, colonnesA;
int lignesB, colonnesB;
const float pas_mm = 1.97;
const int MAX_POS = 100;
int positionsB[MAX_POS][2];
int totalInsertions = 0;
int bouchonsParBoiteB ;
int indexA = 0, indexB = 0;

// === États système ===
bool cycleStarted = false;
bool emergencyActive = false;

// === Anti-rebond ===
unsigned long lastDebounceTime = 0;
const unsigned long debounceDelay = 50;

// === Clignotement LED d’urgence ===
unsigned long lastBlinkTime = 0;
bool ledState = false;
const unsigned long blinkInterval = 500;

// === Fonctions déclarées ===
void initializeHardware();
void homing();
void handleServo();
void emergencyStop();
void beepBuzzer(int duration);
int readIntFromSerial(const char* prompt);
void configureMatrix();
long getPositionX(int l, int c);
long getPositionY(int l, int c);
void runWithEmergencyCheck(AccelStepper &m);
void runDualWithEmergencyCheck(AccelStepper &a, AccelStepper &b);
void waitForStart(const char* msg);
void initialisationWifi();

void setup() {
  Serial.begin(115200);
  initialisationWifi();
  initializeHardware();
  Serial.println("✅ Système prêt. Appuyez sur START (D12) pour configurer et démarrer.");
}

void loop() {
  handleServo();

  if (digitalRead(EMERGENCY_STOP_PIN) == LOW && !emergencyActive) {
    emergencyStop();
  }

  if (emergencyActive) {
    if (millis() - lastBlinkTime >= blinkInterval) {
      ledState = !ledState;
      digitalWrite(LED_PIN, ledState);
      lastBlinkTime = millis();
    }
    if (digitalRead(START_BUTTON) == LOW) {
      delay(200);
      emergencyActive = false;
      digitalWrite(LED_PIN, LOW);
      digitalWrite(LED_BOUCHON, HIGH);
      stepperX.enableOutputs();
      stepperY.enableOutputs();
      stepperZ.enableOutputs();
      Serial.println("✅ Système réactivé !");
      beepBuzzer(200);
    }
    return;
  }

  if (digitalRead(HOMING_BUTTON) == LOW && millis() - lastDebounceTime > debounceDelay) {
    Serial.println("🔄 Calibration en cours...");
    homing();
    Serial.println("✅ Calibration terminée");
    lastDebounceTime = millis();
  }

  if (!cycleStarted && digitalRead(START_BUTTON) == LOW) {
  cycleStarted = true;
  Serial.println("▶️ Cycle démarré");

  // Déplacement initial à X=100, Y=50, Z=0
  long targetX = 15 * 100;  // conversion pour getPositionX logique
  long targetY = 12 * 100;
  stepperX.moveTo(targetX);
  stepperY.moveTo(targetY);
  runDualWithEmergencyCheck(stepperX, stepperY);
  stepperZ.moveTo(0);
  runWithEmergencyCheck(stepperZ);
  Serial.println("📍 Position initiale atteinte : X=15, Y=12, Z=0");

  // Lancement configuration
  configureMatrix();
  indexA = 0;
  indexB = 0;
  digitalWrite(LED_BOUCHON, HIGH);
  delay(300);
}

  if (!cycleStarted) return;

  while (indexA < lignesA * colonnesA) {
    for (int b = 0; b < bouchonsParBoiteB && indexA < lignesA * colonnesA; b++) {
      if (indexB >= totalInsertions) {
        Serial.println("✅ Tous les emplacements de la boîte B actuelle sont utilisés.");
        waitForStart("📦 Insérez une nouvelle boîte B vide et appuyez sur START.");
        indexB = 0;
      }

      Serial.print("🔢 Insertion bouchon #"); Serial.println(indexA);

      // Aller vers A[i][j]
      int iA = indexA / colonnesA;
      int jA = indexA % colonnesA;
      Serial.print("➡️ Aller à A["); Serial.print(iA); Serial.print(","); Serial.print(jA); Serial.println("]");
      stepperX.moveTo(getPositionX(iA, jA) * -1);  // A est à gauche
      stepperY.moveTo(getPositionY(iA, jA));
      runDualWithEmergencyCheck(stepperX, stepperY);

      // Prendre le bouchon
      stepperZ.moveTo(-500); runWithEmergencyCheck(stepperZ);
      myServo.write(0); delay(500);  // Fermer
      stepperZ.moveTo(0); runWithEmergencyCheck(stepperZ);

      // Aller à B[i][j]
      int lB = positionsB[indexB][0];
      int cB = positionsB[indexB][1];
      Serial.print("➡️ Déplacement vers B["); Serial.print(lB); Serial.print(","); Serial.print(cB); Serial.println("]");
      stepperX.moveTo(getPositionX(lB, cB)); stepperY.moveTo(getPositionY(lB, cB));
      runDualWithEmergencyCheck(stepperX, stepperY);

      // Déposer le bouchon
      stepperZ.moveTo(-500); runWithEmergencyCheck(stepperZ);
      myServo.write(90); delay(500);  // Ouvrir
      stepperZ.moveTo(0); runWithEmergencyCheck(stepperZ);

      indexA++;
      indexB++;

      Serial.print("✅ Bouchon inséré: A="); Serial.print(indexA); Serial.print(" B="); Serial.println(indexB);
      delay(300);
    }
  }

  Serial.println("📭 Boîte A vide. Insérez une nouvelle boîte A.");
  waitForStart("🔁 Appuyez sur START une fois la boîte A remplacée.");
  cycleStarted = false;
}

void waitForStart(const char* msg) {
  Serial.println(msg);
  while (digitalRead(START_BUTTON) == HIGH);
  delay(300);
}

int readIntFromSerial(const char* prompt) {
  Serial.print(prompt);
  while (!Serial.available());
  int v = Serial.parseInt();
  Serial.println(v);
  while (Serial.available()) Serial.read();
  return v;
}

void configureMatrix() {
  lignesA = readIntFromSerial("Lignes A: ");
  colonnesA = readIntFromSerial("Colonnes A: ");
  lignesB = readIntFromSerial("Lignes B: ");
  colonnesB = readIntFromSerial("Colonnes B: ");
  bouchonsParBoiteB = config[0].valeur;
  totalInsertions = bouchonsParBoiteB;

  for (int i = 0; i < totalInsertions; i++) {
    bool valid = false;
    while (!valid) {
      Serial.print("Position B[#"); Serial.print(i); Serial.print("] ligne,col: ");
      while (!Serial.available());
      int l = Serial.parseInt();
      while (!Serial.available());
      int c = Serial.parseInt();
      while (Serial.available()) Serial.read();

      if (l >= 0 && l < lignesB && c >= 0 && c < colonnesB) {
        positionsB[i][0] = l;
        positionsB[i][1] = c;
        Serial.print(" -> ("); Serial.print(l); Serial.print(","); Serial.print(c); Serial.println(") ✅");
        valid = true;
      } else {
        Serial.println("❌ Coordonnées hors limites. Réessayez.");
        Serial.print("👉 Boîte B: max lignes = "); Serial.print(lignesB - 1);
        Serial.print(", max colonnes = "); Serial.println(colonnesB - 1);
      }
    }
  }
}


void initializeHardware() {
  stepperX.setMaxSpeed(1000); stepperX.setAcceleration(500);
  stepperY.setMaxSpeed(1000); stepperY.setAcceleration(500);
  stepperZ.setMaxSpeed(1000); stepperZ.setAcceleration(500);

  pinMode(ENDSTOP_X_PIN, INPUT_PULLUP);
  pinMode(ENDSTOP_Y_PIN, INPUT_PULLUP);
  pinMode(ENDSTOP_Z_PIN, INPUT_PULLUP);
  pinMode(HOMING_BUTTON, INPUT_PULLUP);
  pinMode(START_BUTTON, INPUT_PULLUP);
  pinMode(SERVO_BUTTON_PIN, INPUT_PULLUP);
  pinMode(BOUCHON_BUTTON, INPUT_PULLUP);
  pinMode(EMERGENCY_STOP_PIN, INPUT_PULLUP);
  pinMode(LED_PIN, OUTPUT);
  pinMode(LED_BOUCHON, OUTPUT);
  pinMode(BUZZER_PIN, OUTPUT);

  myServo.attach(SERVO_PIN);
  myServo.write(90);  // pince ouverte
}

void homing() {
  while (digitalRead(ENDSTOP_X_PIN)) {
    stepperX.move(-1);
    stepperX.run();
    if (digitalRead(EMERGENCY_STOP_PIN) == LOW) {
      emergencyStop();
      return;
    }
  }
  stepperX.setCurrentPosition(0);

  while (digitalRead(ENDSTOP_Y_PIN)) {
    stepperY.move(-1);
    stepperY.run();
    if (digitalRead(EMERGENCY_STOP_PIN) == LOW) {
      emergencyStop();
      return;
    }
  }
  stepperY.setCurrentPosition(0);

  while (digitalRead(ENDSTOP_Z_PIN)) {
    stepperZ.move(-1);
    stepperZ.run();
    if (digitalRead(EMERGENCY_STOP_PIN) == LOW) {
      emergencyStop();
      return;
    }
  }
  stepperZ.setCurrentPosition(0);
}

void runWithEmergencyCheck(AccelStepper &m) {
  while (m.distanceToGo()) {
    if (digitalRead(EMERGENCY_STOP_PIN) == LOW) { emergencyStop(); return; }
    if (emergencyActive) return;
    m.run();
  }
}

void runDualWithEmergencyCheck(AccelStepper &a, AccelStepper &b) {
  while (a.distanceToGo() || b.distanceToGo()) {
    if (digitalRead(EMERGENCY_STOP_PIN) == LOW) { emergencyStop(); return; }
    if (emergencyActive) return;
    a.run(); b.run();
  }
}

void handleServo() {
  if (emergencyActive) return;
  if (digitalRead(SERVO_BUTTON_PIN) == LOW) {
    delay(100);
    servoState = !servoState;
    myServo.write(servoState ? 90 : 0);
    Serial.println(servoState ? "Servo ouvert" : "Servo fermé");
    while (digitalRead(SERVO_BUTTON_PIN) == LOW);
  }
}

void emergencyStop() {
  emergencyActive = true;
  Serial.println("🛑 ARRÊT D'URGENCE ACTIVÉ !");
  beepBuzzer(200);
  stepperX.stop(); stepperY.stop(); stepperZ.stop();
  stepperX.disableOutputs(); stepperY.disableOutputs(); stepperZ.disableOutputs();
  myServo.write(90);
  digitalWrite(LED_BOUCHON, LOW);
}

void beepBuzzer(int d) {
  digitalWrite(BUZZER_PIN, HIGH);
  delay(d);
  digitalWrite(BUZZER_PIN, LOW);
}

// === Conversion coordonnées vers distances moteur ===
long getPositionX(int l, int c) {
  return (long)(c * pas_mm * 100);  // en 1/100e mm si pas_mm = 1.97
}

long getPositionY(int l, int c) {
  return (long)(l * pas_mm * 100);
}

void initialisationWifi(){
  WiFi.begin(WIFI_SSID, WIFI_PASSWORD, WIFI_CHANNEL);
  Serial.print("Connecting to WiFi ");
  Serial.print(WIFI_SSID);
  // Wait for connection
  while (WiFi.status() != WL_CONNECTED) {
      delay(100);
      Serial.print(".");
  }
  Serial.println(" Connected!");

  Serial.print("IP address: ");
  Serial.println(WiFi.localIP());
  // Page HTML (formulaire)
  server.on("/", HTTP_GET, [](AsyncWebServerRequest *request){
      String html = R"rawliteral(
      <!DOCTYPE html>
      <html>
      <head>
          <meta charset="UTF-8">
          <title>Configuration</title>
          <style>
          body {
              font-family: Arial, sans-serif;
              background-color: #f7f7f7;
              padding: 20px;
          }
          h2 {
              color: #333;
          }
          form {
              max-width: 400px;
              margin: auto;
              background: white;
              padding: 20px;
              border-radius: 10px;
              box-shadow: 0 2px 10px rgba(0, 0, 0, 0.1);
          }
          label {
              display: block;
              margin-top: 15px;
              font-weight: bold;
          }
          input[type='number'] {
              width: 100%;
              padding: 8px;
              margin-top: 5px;
              border: 1px solid #ccc;
              border-radius: 5px;
              box-sizing: border-box;
          }
          input[type='submit'] {
              margin-top: 20px;
              width: 100%;
              padding: 10px;
              background-color: #4CAF50;
              color: white;
              border: none;
              border-radius: 5px;
              font-size: 16px;
              cursor: pointer;
          }
          input[type='submit']:hover {
              background-color: #45a049;
          }
          </style>
      </head>
      <body>
          <h2>Configuration du système</h2>
          <form action="/save" method="get">
      )rawliteral";
  
      for (int i = 0; i < NB_PARAMS; ++i) {
      html += "<label for='";
      html += config[i].nom;
      html += "'>";
      html += config[i].nom;
      html += "</label><input type='number' name='";
      html += config[i].nom;
      html += "' value='";
      html += config[i].valeur;
      html += "'>\n";
      }
  
      html += R"rawliteral(
          <input type="submit" value="Enregistrer">
          </form>
      </body>
      </html>
      )rawliteral";
  
      request->send(200, "text/html", html);
  });
  

  // Traitement des données reçues
  server.on("/save", HTTP_GET, [](AsyncWebServerRequest *request){
      
      for (int i = 0; i < NB_PARAMS; ++i) {
      if (request->hasParam(config[i].nom)) {
          config[i].valeur = request->getParam(config[i].nom)->value().toInt();
      }
      }
     
      Serial.println("== Configuration mise à jour ==");
      for (int i = 0; i < NB_PARAMS; ++i) {
      Serial.print( config[i].nom );
      Serial.print(": ");
      Serial.println( config[i].valeur );

      }
  
      request->send(200, "text/plain", "Configuration enregistrée !");
  });
  server.begin();
  
  }