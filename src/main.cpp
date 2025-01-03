//Abhaengigkeiten
#include <Arduino.h>
#include <math.h>
#include <stdio.h>

//PIN Deklarationen
#define MOTOR_LEFT_IN1 9
#define MOTOR_LEFT_IN2 10
#define MOTOR_LEFT_IN3 11
#define MOTOR_LEFT_IN4 12

#define MOTOR_RIGHT_IN1 4
#define MOTOR_RIGHT_IN2 5
#define MOTOR_RIGHT_IN3 6
#define MOTOR_RIGHT_IN4 7


//Definition des eigener Datentypen mittels struct

//Motordaten struct beinhaltet alle benötigen Motorvariablen pro Motor (Hat 2 InOut)
struct MOTOR_DATA {
  int IN1;
  int IN2;
  int IN3;
  int IN4;
  int stepNumber; //InOut
  float currentPosition; //InOut
  int maxAmountOfSteps;
  float translation;
};

//Zielpunkt struct beinhaltet Zielkoordinaten, Ausfuehrungszeit, Be- und Entschleunigungsdaten (Kein InOut)
struct DESTINATION_DATA {
  float xValue;
  float yValue;
  float travelTime;
  float timeDelayP0;
  float timeDelayP1;
  float timeDelayP2;
  float timeDelayP3;
  float timeDelayP4;
};

//Roboterdaten struct beinhaltet alle nicht-aenderden Vektor und Laengendaten (KeinInOut)
struct ROBOT_DATA {
  float supportVectora1Val;
  float supportVectora11Val;
  float directionVectora2Val;
  float directionVectorb2Val;
  float supportVectora1X;
  float supportVectora1Y;
  float supportVectora11X;
  float supportVectora11Y;
};

//Erstellen der Structs
MOTOR_DATA MOTOR_LEFT;
MOTOR_DATA MOTOR_RIGHT;
ROBOT_DATA ROBOT;
DESTINATION_DATA DESTINATION;

//Konvertierungsalgorihmus von Vektor zu Motorwinkel
void conversionAlg(float xValue, float yValue, float angles[2], struct ROBOT_DATA &robot) {

  //Definition PI
  const float pi = 3.1415926;

  //Berechnung des Betrages vom Zielvektor
  float destinationVectorVal = sqrt(pow(xValue, 2)+pow(yValue, 2));

  //Berechnung der unterstuezenden Vektoren C1 und C2
  //Zielvektor berechnen, mit Pythagroassatz den Betrag des Vektors ermitteln

  float temporaryC1XVal = xValue - ROBOT.supportVectora1X;
  float temporaryC1YVal = yValue - ROBOT.supportVectora1Y;

  float supportVectorC1 = sqrt(pow(temporaryC1XVal, 2)+pow(temporaryC1YVal, 2));
    
  float temporaryC2XVal = xValue - ROBOT.supportVectora11X;
  float temporaryC2YVal = yValue - ROBOT.supportVectora11Y;

  float supportVectorC2 = sqrt(pow(temporaryC2XVal, 2)+pow(temporaryC2YVal, 2));

  //Berechnung von Beta 1 (rechter Motorwinkel)
  //Verwendete Formeln: Kosinussatz (Umformung nach Winkel), Radient zu Grad

  float sectorBeta11 = (acosf((pow(ROBOT.directionVectora2Val, 2)+pow(supportVectorC1, 2)-pow(ROBOT.directionVectorb2Val, 2))/(2 * ROBOT.directionVectora2Val * supportVectorC1)))*180.0f/pi;

  float sectorBeta12 = (acosf((pow(ROBOT.supportVectora1Val, 2)+pow(supportVectorC1, 2)-pow(destinationVectorVal, 2))/(2 * ROBOT.supportVectora1Val * supportVectorC1)))*180.0f/pi;

  float beta1 = sectorBeta11 + sectorBeta12;

  //Berechnung von Beta 2 (linker Motorwinkel)
  //Verwendete Formeln: Kosinussatz (Umformung nach Winkel), Radient zu Grad

  float sectorBeta21 = (acosf((pow(ROBOT.directionVectora2Val, 2)+pow(supportVectorC2, 2)-pow(ROBOT.directionVectorb2Val, 2))/(2 * ROBOT.directionVectora2Val * supportVectorC2)))*180.0f/pi;

  float sectorBeta22 = (acosf((pow(ROBOT.supportVectora11Val, 2)+pow(supportVectorC2, 2)-pow(destinationVectorVal, 2))/(2 * ROBOT.supportVectora11Val * supportVectorC2)))*180.0f/pi;

  float beta2 = sectorBeta21 + sectorBeta22;

  //Ergebnis weitergeben

  angles[0] = beta2;
  angles[1] = beta1;
}

//Funktion um einen Schritt zu machen
//Der Parameter direction gibt die Drehrichtung an
void oneStep(bool direction, MOTOR_DATA &motor_data) {

  //Einen Schritt machen
  //stepNumber prüfen und anhand von dieser nächsten oder vorherigen Schritt tätigen
  if (direction == true) {

    switch(motor_data.stepNumber){
      case 0:
      digitalWrite(motor_data.IN1, HIGH);
      digitalWrite(motor_data.IN2, LOW);
      digitalWrite(motor_data.IN3, LOW);
      digitalWrite(motor_data.IN4, LOW);
      break;
      case 1:
      digitalWrite(motor_data.IN1, LOW);
      digitalWrite(motor_data.IN2, HIGH);
      digitalWrite(motor_data.IN3, LOW);
      digitalWrite(motor_data.IN4, LOW);
      break;
      case 2:
      digitalWrite(motor_data.IN1, LOW);
      digitalWrite(motor_data.IN2, LOW);
      digitalWrite(motor_data.IN3, HIGH);
      digitalWrite(motor_data.IN4, LOW);
      break;
      case 3:
      digitalWrite(motor_data.IN1, LOW);
      digitalWrite(motor_data.IN2, LOW);
      digitalWrite(motor_data.IN3, LOW);
      digitalWrite(motor_data.IN4, HIGH);
      break;
    } 

  }
  
  if (direction == false) {

    switch(motor_data.stepNumber){
      case 0:
      digitalWrite(motor_data.IN1, LOW);
      digitalWrite(motor_data.IN2, LOW);
      digitalWrite(motor_data.IN3, LOW);
      digitalWrite(motor_data.IN4, HIGH);
      break;
      case 1:
      digitalWrite(motor_data.IN1, LOW);
      digitalWrite(motor_data.IN2, LOW);
      digitalWrite(motor_data.IN3, HIGH);
      digitalWrite(motor_data.IN4, LOW);
      break;
      case 2:
      digitalWrite(motor_data.IN1, LOW);
      digitalWrite(motor_data.IN2, HIGH);
      digitalWrite(motor_data.IN3, LOW);
      digitalWrite(motor_data.IN4, LOW);
      break;
      case 3:
      digitalWrite(motor_data.IN1, HIGH);
      digitalWrite(motor_data.IN2, LOW);
      digitalWrite(motor_data.IN3, LOW);
      digitalWrite(motor_data.IN4, LOW);
      break;
    }
  }

  //Merkervariable um 1 erhöhen, damit jetziger Schritt gespeichert wird
  motor_data.stepNumber++;

  //Falls die Merkervariable 4 beträgt, muss diese auf 0 gesetzt werden um mit dem "ersten" Motorschritt weiterzumachen
  if (motor_data.stepNumber > 3) {

    motor_data.stepNumber = 0;
  }

}

//Konvertierung von Grad zu Motorschritten
float conversion(float input, int maxInput, int maxOutput, float translation) {
  //Simpler Dreisatz, welcher den die Übersetzung des Zahnrades inkludiert
  return (input / maxInput) * maxOutput * translation;
}

//Funktion den Motor linear zu verfahren
int moveL(struct DESTINATION_DATA destination, struct MOTOR_DATA &motor_data_right, struct MOTOR_DATA &motor_data_left, struct ROBOT_DATA &robot, bool debugMode) {

  //Speicherort für Winkel kreieren
  float angles[2] = {0.0, 0.0};

  //Debug
  if (debugMode == true) {
    Serial.println("Linker Motor vor Berechnung:");
    Serial.println(angles[0]);
    Serial.println("Rechter Motor vor Berechnung:");
    Serial.println(angles[1]);
  }

  //Winkel berechnen
  conversionAlg(destination.xValue, destination.yValue, angles, robot);

  //Debug
  if (debugMode == true) {
    Serial.println("Linker Motor nach Berechnung:");
    Serial.println(angles[0]);
    Serial.println("Rechter Motor nach Berechnung:");
    Serial.println(angles[1]);
  }

  //Schutzmechanismus
  //Falls Zielposition nicht erreichbar ist, abbrechen
  if (angles[0] != angles[0] || angles[1] != angles[1]) {
    if (debugMode == true) {
      Serial.print("Position ist nicht erreichbar!");
    }
    return 1;
  }

  //Speicherort für Winkel in Schritten kreieren
  float anglesInSteps[2] = {0.0, 0.0};

  //Winkel in Schritten umrechnen
  anglesInSteps[0] = conversion(angles[0], 360, motor_data_left.maxAmountOfSteps, motor_data_left.translation);
  anglesInSteps[1] = conversion(angles[1], 360, motor_data_right.maxAmountOfSteps, motor_data_right.translation);

  //Debug
  if (debugMode == true) {
    Serial.println("Linker Motor Schritte nach Berechnung:");
    Serial.println(anglesInSteps[0]);
    Serial.println("Rechter Motor Schritte nach Berechnung:");
    Serial.println(anglesInSteps[1]);
  }
  //Speicherort für wieviel der Motor drehen muss kreieren
  float turnMotorSteps[2] = {0.0, 0.0};

  //Berechnen um wie viel der Motor sich drehen muss
  turnMotorSteps[0] = motor_data_left.currentPosition - anglesInSteps[0];
  turnMotorSteps[1] = motor_data_right.currentPosition - anglesInSteps[1];

  //Debug
  if (debugMode == true) {
    Serial.println("Wieviel drehen links vor aufbereitung:");
    Serial.println(turnMotorSteps[0]);
    Serial.println("Wieviel drehen rechts vor aufbereitung:");
    Serial.println(turnMotorSteps[1]);  
  }

  //Drehrichtungparameter erstellen
  bool turnDirectionMotorLeft = true;
  bool turnDirectionMotorRight = true;

  //Daten aufbereiten Motor Links
  if (turnMotorSteps[0] < 0) {
    turnDirectionMotorLeft = true;
    turnMotorSteps[0] = turnMotorSteps[0] * -1;
  } else {
    turnDirectionMotorLeft = false;
  }

  //Daten aufbereiten Motor Rechts
  if (turnMotorSteps[1] < 0) {
    turnDirectionMotorRight = false;
    turnMotorSteps[1] = turnMotorSteps[1] * -1;
  } else {
    turnDirectionMotorRight = true;
  }

  //Debug
  if (debugMode == true) {
    Serial.println("Wieviel drehen links nach aufbereitung:");
    Serial.println(turnMotorSteps[0]);
    Serial.println("Richtung:");
    Serial.println(turnDirectionMotorLeft);
    Serial.println("Wieviel drehen rechts nach aufbereitung:");
    Serial.println(turnMotorSteps[1]);  
    Serial.println("Richtung:");
    Serial.println(turnDirectionMotorRight);
  }


  //Motor Linear drehen
  float maxSteps = 0.0f; //Speichert die Anzahl maximaler Schritte, welche getaetigt werden muessen
  int numberStepsLeft = 0; //Speichert die aktuelle Anzahl von Schritten, welche beim Schrittmotor links getaetigt wurden
  int numberStepsRight = 0; //Speichert die aktuelle Anzahl von Schritten, welche beim Schrittmotor recht getaetigt wurden
  float stepsRightBigger = false; //Speichert, ob links oder recht mehr Schritte getaetigt werden muessen (Wichtig für die kubische Bezierkurve)

  //maxSteps auswerten
  if (turnMotorSteps[1] > turnMotorSteps[0]) {
    maxSteps = turnMotorSteps[1];
    stepsRightBigger = true;
  } else {
    maxSteps = turnMotorSteps[0];
  }

  //Schutzmechanismus
  //Falls keine Schritte getaetigt werden müssen ist die Zielposition schon erreicht
  if (turnMotorSteps[0] <= 0 && turnMotorSteps[1] <= 0) {
    return 0;
  }

  //Zeitverzögerung fuer insgesamte Bewegungszeit berechnen
  float averageTimeDelay = (destination.travelTime / maxSteps) * 1000.0f;
  //Durchschnittliche Zeitverzögerung für die Bezierkurve berechnen
  float originalAverageDelay = (destination.timeDelayP0 + destination.timeDelayP1 + destination.timeDelayP2 + destination.timeDelayP3 + destination.timeDelayP4) / 5.0f;
  //Skalierungsfaktor ist das Verhältnis zwischen
  float scaleFactor = averageTimeDelay / originalAverageDelay;

  //Schutzmechanismus
  //Falls der Skalierungsfaktor unter 1 liegt, kann es dazu kommen, dass die Verzögerung von Motorschritten unter 2ms liegt
  if (scaleFactor < 1.0f) {
    scaleFactor = 1.0f;
  }

  //Verhältnis zwischen den Schritten und der Anzahl maximalen Schritten
  float ratioToTotalLeft = turnMotorSteps[0] / maxSteps; 
  float ratioToTotalRight = turnMotorSteps[1] / maxSteps; 


  //Debug
  if (debugMode == true) {
    Serial.println("Verhältnis Links");
    Serial.println(ratioToTotalLeft);
    Serial.println("Verhältnis Rechts");
    Serial.println(ratioToTotalRight);
    Serial.println("Skalierungsfaktor");
    Serial.println(scaleFactor);
  }

  //Erstellung Akkumulatorvariable
  float leftAccumulator = 0.0f;
  float rightAccumulator = 0.0f;

  //For-Schleife zum machen der Schritte (Limit --> Maximale Anzahl Schritte)
  for (int i = 0; i < maxSteps; i++) {

    //Erstellung Faktor t für Bezier Kurve
    float t = 0.0;

    //Faktor t für die Bezier Kurve berechnen
    //Jetzigen Schrittwert im Verhältnis zu der maximalen Anzahl Schritte skalieren zwischen 0 und 1
    if (stepsRightBigger) {
      t = numberStepsRight / (maxSteps - 1);
    } else {
      t = numberStepsLeft / (maxSteps - 1);
    }

    //Mittels quadratischer Bezierkurve (4. Grad) die Zeitverzögerung berechnen
    float delayTime = (pow((1 - t), 4) * destination.timeDelayP0 + 4 * pow((1 - t), 3) * t * destination.timeDelayP1 + 6 * pow((1 - t), 2) * pow(t, 2) * destination.timeDelayP2 + 4 * (1 - t) * pow(t, 3) * destination.timeDelayP3 + pow(t, 4) * destination.timeDelayP4) * scaleFactor;

    //Debug
    if (debugMode) {
      Serial.println(delayTime);
    }

    //Das Verhältnis zur Akkumulatorvariable addieren
    leftAccumulator += ratioToTotalLeft;
    rightAccumulator += ratioToTotalRight;

    //Sobald diese erreicht ist, einen Schritt tätigen, dabei die Variable zurücksetzen und einen Schritt tätigen
    if (leftAccumulator >= 1.0f){
        oneStep(turnDirectionMotorLeft, motor_data_left);
        leftAccumulator -= 1.0f;
        numberStepsLeft++;
    }
    if (rightAccumulator >= 1.0f){
        oneStep(turnDirectionMotorRight, motor_data_right);
        rightAccumulator -= 1.0f;
        numberStepsRight++;
    }

    //Zeitverzögerung verwenden
    delay((int)delayTime);
  }
  
  //Rest nachkorrigieren
  if (rightAccumulator > 0.5f) {
    oneStep(turnDirectionMotorLeft, motor_data_left);
    numberStepsLeft++;
  }

  if (leftAccumulator > 0.5f) {
    oneStep(turnDirectionMotorRight, motor_data_right);
    numberStepsRight++;
  }

  //Position anpassen
  if (turnDirectionMotorLeft == true) {
    motor_data_left.currentPosition += numberStepsLeft;
  } else {
    motor_data_left.currentPosition -= numberStepsLeft;
  }

  if (turnDirectionMotorRight == true) {
    motor_data_right.currentPosition -= numberStepsRight;
  } else {
    motor_data_right.currentPosition += numberStepsRight;
  }


  //Debug
  if (debugMode == true) {
    Serial.println("Neue aktuelle position links:");
    Serial.println(motor_data_left.currentPosition);
    Serial.println("Neue aktuelle position rechts:");
    Serial.println(motor_data_right.currentPosition);
  }

  //Aktion erfolgreich
  return 0;

}

void setup() {

  Serial.begin(9600);

  //Definition der Pins
  pinMode(MOTOR_LEFT_IN1, OUTPUT);
  pinMode(MOTOR_LEFT_IN2, OUTPUT);
  pinMode(MOTOR_LEFT_IN3, OUTPUT);
  pinMode(MOTOR_LEFT_IN4, OUTPUT);

  pinMode(MOTOR_RIGHT_IN1, OUTPUT);
  pinMode(MOTOR_RIGHT_IN2, OUTPUT);
  pinMode(MOTOR_RIGHT_IN3, OUTPUT);
  pinMode(MOTOR_RIGHT_IN4, OUTPUT);



  //Robot Struct initialisieren
  ROBOT.supportVectora1Val = 3.5f;
  ROBOT.supportVectora11Val = 3.5f;
  ROBOT.directionVectora2Val = 8.0f;
  ROBOT.directionVectorb2Val = 16.0f;
  ROBOT.supportVectora1X = 3.5f;
  ROBOT.supportVectora1Y = 0.0f;
  ROBOT.supportVectora11X = -3.5f;
  ROBOT.supportVectora11Y = 0.0f;

  //Motor left Struct initialisieren
  MOTOR_LEFT.IN1 = MOTOR_LEFT_IN1;
  MOTOR_LEFT.IN2 = MOTOR_LEFT_IN2;
  MOTOR_LEFT.IN3 = MOTOR_LEFT_IN3;
  MOTOR_LEFT.IN4 = MOTOR_LEFT_IN4;
  MOTOR_LEFT.stepNumber = 0;
  MOTOR_LEFT.currentPosition = 1638.4f;
  MOTOR_LEFT.maxAmountOfSteps = 2048;
  MOTOR_LEFT.translation = 1.6f;

  //Motor right Struct initialisieren
  MOTOR_RIGHT.IN1 = MOTOR_RIGHT_IN1;
  MOTOR_RIGHT.IN2 = MOTOR_RIGHT_IN2;
  MOTOR_RIGHT.IN3 = MOTOR_RIGHT_IN3;
  MOTOR_RIGHT.IN4 = MOTOR_RIGHT_IN4;
  MOTOR_RIGHT.stepNumber = 0;
  MOTOR_RIGHT.currentPosition = 1638.4f;
  MOTOR_RIGHT.maxAmountOfSteps = 2048;
  MOTOR_RIGHT.translation = 1.6f;

  //Zielpunkt Struct initialisieren
  DESTINATION.xValue = -10.0f;
  DESTINATION.yValue = 10.0f;
  DESTINATION.travelTime = 0.0f;
  DESTINATION.timeDelayP0 = 5.0f;
  DESTINATION.timeDelayP1 = 2.0f;
  DESTINATION.timeDelayP2 = 1.7f;
  DESTINATION.timeDelayP3 = 2.0f;
  DESTINATION.timeDelayP4 = 5.0f;

  moveL(DESTINATION, MOTOR_RIGHT, MOTOR_LEFT, ROBOT, true);

}

void loop() {

  DESTINATION.xValue = -10.0f;
  DESTINATION.yValue = 17.0f;

  moveL(DESTINATION, MOTOR_RIGHT, MOTOR_LEFT, ROBOT, true);

  DESTINATION.xValue = -10.0f;
  DESTINATION.yValue = 10.0f;

  moveL(DESTINATION, MOTOR_RIGHT, MOTOR_LEFT, ROBOT, true);

  DESTINATION.xValue = 10.0f;
  DESTINATION.yValue = 10.0f;

  moveL(DESTINATION, MOTOR_RIGHT, MOTOR_LEFT, ROBOT, true);

  DESTINATION.xValue = 10.0f;
  DESTINATION.yValue = 17.0f;

  moveL(DESTINATION, MOTOR_RIGHT, MOTOR_LEFT, ROBOT, true);

  DESTINATION.xValue = 10.0f;
  DESTINATION.yValue = 10.0f;

  moveL(DESTINATION, MOTOR_RIGHT, MOTOR_LEFT, ROBOT, true);

  DESTINATION.xValue = -10.0f;
  DESTINATION.yValue = 10.0f;

  moveL(DESTINATION, MOTOR_RIGHT, MOTOR_LEFT, ROBOT, true);



}
