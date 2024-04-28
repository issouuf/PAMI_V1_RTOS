#include <Wire.h>
#include <Arduino.h>  // Inclure pour Serial
#include <math.h>
#include "Codeurs.h"
#include "Vl53l0x.h"
#include "rgb_lcd.h"
#include "FreeRTOS.h"
#include "task.h"

Vl53l0x capteur1;
Codeurs codeurs;
rgb_lcd lcd;


TaskHandle_t Handle_aTask;
TaskHandle_t Handle_bTask;


#define BACKWARD 0x1
#define FORWARD 0x0

int _address = 0x10;
int32_t _gauche, _droit;
int16_t _g16, _d16;
static const int16_t MAX = 16384;
int32_t codeurGauche, codeurDroit;
int pinDirectionDroit = PA15;
int pinPWMDroit = PA13;
int pinDirectionGauche = PA14;
int pinPWMGauche = PA12;
const int mesureVbat = PB3;
const int TOR1 = 2;
const int TOR2 = 3;
int etat = 0;
int TOR1_precedent = LOW;  //laisser en LOW


/// Variable utiliser pour l'asservissemnt de direction
float distance = 0, erreur, commande, commandeTraiter, commandeComp = 0.1, vitesse, Kalpha = 50, PWM, PWMD, PWMG, consDistance = 0;
float m_distance;
float angle_cons_droit, angle_cons_gauche;
float rayonRoue = 60.0;
float vit_ang = 50.0;

float Te = 10;
float Tc = 100;



void controle(void *parameters) {
  TickType_t xLastWakeTime;
  xLastWakeTime = xTaskGetTickCount();

  while (1) {
    int etat_TOR1 = digitalRead(TOR1);
    int etat_TOR2 = digitalRead(TOR2);
    float vbat = analogRead(mesureVbat);
    float tensionBat = (vbat / 895) * 3.3;
    tensionBat = tensionBat * (7.2 / 3.3);

    lcd.setCursor(7, 0);
    lcd.print("vbat: ");
    lcd.setCursor(12, 0);
    lcd.print(tensionBat);


    // if (etat_TOR1 != TOR1_precedent) {
    //   if (etat_TOR1 == LOW) {
    //     Serial.println("bp1 desactivé");

    //   }else{
    //     Serial.println("bp1 activé");
    //   }
    // }

    switch (etat) {
      case 0:
        //lcd.setRGB(0, 0, 255);
        if (etat_TOR1 != TOR1_precedent) {
          if (etat_TOR1 == LOW) {
            etat = 1;
          }
        }
        break;
      case 1:
        //lcd.setRGB(0, 255, 0);
        X_Y_Theta(2000, 1200, -120);
        etat = 0;
        break;
    }
    TOR1_precedent = etat_TOR1;
    Serial.println("Je suis dans controle!");

    vTaskDelayUntil(&xLastWakeTime, pdMS_TO_TICKS(Te));
  }
}


void capteurs(void *parameters) {
  TickType_t xLastWakeTime;
  xLastWakeTime = xTaskGetTickCount();

  VL53L0X_RangingMeasurementData_t rangingMeasurementData;
  while (1) {

    
    // capteur1.performContinuousRangingMeasurement(&rangingMeasurementData);
    // Serial.println(rangingMeasurementData.RangeMilliMeter);
    Serial.println("Je suis passé par la !");

    vTaskDelayUntil(&xLastWakeTime, pdMS_TO_TICKS(Tc));
  }
}

////////////////////////////////////// Moteur Droit/////////////////////////////////////////////////////////////////////////////////////////////////////////
//////////////////////// Initialisation du moteur Droit/////////////////////////////////////////////////////////////////////////////////////////////////////
void initMoteurDroit() {
  pinMode(pinPWMDroit, OUTPUT);
  pinMode(pinDirectionDroit, OUTPUT);
}
//////////////////////// Permet de pouvoir déterminer dans un premier comment fonctionne les  moteurs//////////////////////////////////////////////////////
void controleMoteurDroit(int PWM) {
  if (PWM > 0) {
    digitalWrite(pinDirectionDroit, 0);
  }
  if (PWM < 0) {
    digitalWrite(pinDirectionDroit, 1);
  }
  PWM = abs(PWM);
  analogWrite(pinPWMGauche, PWM);
  analogWrite(pinPWMDroit, PWM);
}
////////////////////////////////////// Moteur GAUCHE/////////////////////////////////////////////////////////////////////////////////////////////////////////
//////////////////////// Initialisation du moteur GAUCHE/////////////////////////////////////////////////////////////////////////////////////////////////////
void initMoteurGauche() {
  pinMode(pinPWMGauche, OUTPUT);
  pinMode(pinDirectionGauche, OUTPUT);
}
//////////////////////// Permet de pouvoir déterminer dans un premier comment fonctionne les  moteurs//////////////////////////////////////////////////////
void controleMoteurGauche(int PWM) {
  if (PWM > 0) {
    digitalWrite(pinDirectionGauche, 0);
  }
  if (PWM < 0) {
    digitalWrite(pinDirectionGauche, 1);
  }
  PWM = abs(PWM);
  analogWrite(pinPWMGauche, PWM);
}
void asservissement(float cons) {
  // Serial.println(PWM);
  codeurs.reset();
  delay(1000);
  do {
    codeurs.read(codeurGauche, codeurDroit);
    distance = (codeurGauche + codeurDroit) / 2.0;
    distance = distance * rayonRoue * 2.0 / 2048.0;
    erreur = (cons)-distance;  // Calcul de l'erreur par rapport a la consigne + une consigne d'équilibre due fait que le centre de gravité du robot n'est pas placé correctement
    commande = erreur * 0.25;  // Calcul de la commande permettant de savoir si on doit faire tourner le moteur plus ou moins fort
    if (commande < 0)          // si la commande est négatif, cela signifie qu'on doit faire tourner les moteurs dans un sens spécifiquement
    {
      commandeTraiter = commande - commandeComp;  // Etant donner que les moteur ont physiquement, un jeu et un couple de frottement sec qui doit être éliminer via un offset de la commande
      // PWMD = 10;
      // PWMG = 0;
    }
    if (commande > 0) {
      commandeTraiter = commande + commandeComp;
      // PWMG = 10;
      // PWMD = 0;
    }
    if (commandeTraiter > 1)  // Si la commande dépasse la saturation, on plafonne celle ci a une valeur fixe, due fait que le hacheur 4 quadrant a part son architecture des condensateurs de boostrap, qui doivent etre charger un peu, permettant le bon fonctionnement des transistor
    {
      commandeTraiter = 1;
      PWMD = 20;
      PWMG = 0;
    } else if (commandeTraiter < -1) {
      commandeTraiter = -1;
      PWMG = 20;
      PWMD = 0;
    }
    // Serial.println(erreur);
    // Serial.printf("avance\n");
    vitesse = 0.5 + commandeTraiter;  // Utilisant une commande unipolaire, 0.5 correspond au fait que les deux ponts du hacheur 4 quadrant donne la meme tension, ce qui fait qu'au bornes du moteur on a une tension = 0
    PWM = vitesse * Kalpha;           // On effectue une mise a l'échelle, J'ai décider de travailler sur 12bits
    controleMoteurGauche(PWM + PWMG);
    controleMoteurDroit(PWM - PWMD);
  } while ((erreur <= -2) || (erreur > 5));
}
void rotation(float angle_cons) {
  codeurs.reset();
  delay(1000);
  float erreur_moyenne;
  if (angle_cons > 0) {
    angle_cons = angle_cons + 20.0;
    angle_cons_droit = angle_cons / 2.0;
    angle_cons_gauche = -1 * angle_cons_droit;
  }
  if (angle_cons < 0) {
    angle_cons = angle_cons - 20.0;
    angle_cons_droit = angle_cons / 2.0;
    angle_cons_gauche = -1 * angle_cons_droit;
  }
  do {
    codeurs.read(codeurGauche, codeurDroit);
    float erreur_anglegauche = (codeurGauche * rayonRoue * 2.0 / 2048.0) - (2.0 * 3.1415 * 52.0 * angle_cons_droit / 360.0);
    float erreur_angleDroit = (codeurDroit * rayonRoue * 2.0 / 2048.0) - (2.0 * 3.1415 * 52.0 * angle_cons_gauche / 360.0);
    erreur_moyenne = (erreur_anglegauche - erreur_angleDroit) / 2.0;
    // controleMoteurDroit(50);a
    // Serial.print(" erreur_moyenne ::: ");
    // Serial.print(erreur_moyenne);
    // Serial.print(" Errer dRoit ::: ");
    // Serial.print(erreur_angleDroit);
    // Serial.print("  gauche");
    // Serial.println(erreur_anglegauche);
    if (angle_cons >= 0) {
      if (erreur_moyenne < 0) {
        controleMoteurDroit(-vit_ang - (commandeComp * 255.0));
        controleMoteurGauche(vit_ang + (commandeComp * 255.0));
      }
      if (erreur_moyenne > 0) {
        controleMoteurDroit(0);
        controleMoteurGauche(0);
      }
      // if ((erreur_moyenne > 5)) {
      //   controleMoteurDroit(vit_ang + (commandeComp * 255.0));
      //   controleMoteurGauche(-vit_ang - (commandeComp * 255.0));
      // }
    } else {
      if (erreur_moyenne > 0) {
        controleMoteurDroit(20 + (commandeComp * 255.0));
        controleMoteurGauche(-20 - (commandeComp * 255.0));
      }
      if (erreur_moyenne < 0) {
        controleMoteurDroit(0);
        controleMoteurGauche(0);
      }
      // if ((erreur_moyenne < 5)) {
      //   controleMoteurDroit(-20 - (commandeComp * 255.0));
      //   controleMoteurGauche(20 + (commandeComp * 255.0));
      // }
    }
  } while ((erreur_moyenne < -2) || (erreur_moyenne > 1.5));
}
void X_Y_Theta(float x, float y, float theta) {
  float m_newX = x;
  float m_newY = y;
  float m_newTheta = theta;
  m_distance = sqrt(pow(m_newX - 1500, 2) + pow(m_newY - 1500, 2));
  // m_distance = m_distance * 47.74648;
  //calcul de l'angle en radians
  float res = (m_newY - 2000) / (m_newX - 2000);
  // Serial.println((res));
  float angle = atan2(m_newY - 2000.0, m_newX - 2000.0);

  rotation(degrees(angle));
  controleMoteurDroit(0);
  controleMoteurGauche(0);

  asservissement(m_distance);
  controleMoteurDroit(0);
  controleMoteurGauche(0); 
}


void setup() {

  pinMode(TOR1, INPUT_PULLUP);
  pinMode(TOR2, INPUT_PULLUP);
  pinMode(mesureVbat, INPUT);
  Serial.begin(115200);
  lcd.begin(16, 2);
  codeurs.begin(false);
  initMoteurDroit();
  initMoteurGauche();
  lcd.clear();
  lcd.setRGB(0, 0, 255);
  lcd.setCursor(0, 0);
  lcd.print("PAMI 1");
  lcd.setCursor(0, 1);
  lcd.print("bp1 pour start");

  //configuration capteur TOF
  //VL53L0X_Error status = VL53L0X_ERROR_NONE;
  // status = capteur1.begin(I2C_DEFAULT_ADDR, false);  //true
  capteur1.begin(I2C_DEFAULT_ADDR, false);
  capteur1.continuousRangingInit();
  // if (VL53L0X_ERROR_NONE != status) {
  //   Serial.println("start vl53l0x mesurement failed!");
  //   capteur1.printPalError(status);
  //   while (1)
  //     ;
  // }
  // capteur1.continuousRangingInit();
  // if (VL53L0X_ERROR_NONE != status) {
  //   Serial.println("start vl53l0x mesurement failed!");
  //   capteur1.printPalError(status);
  //   while (1);
  //}

  xTaskCreate(controle,    "controle",  1024,NULL,tskIDLE_PRIORITY + 1, &Handle_aTask);
  xTaskCreate(capteurs,    "capteurs",  1024,NULL,tskIDLE_PRIORITY + 2, &Handle_bTask);
}



void loop() {
    taskYIELD();

}
