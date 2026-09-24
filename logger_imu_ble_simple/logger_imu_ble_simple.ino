// =================================================
// DIAGNOSTIC MINIMAL I2C — XIAO nRF52840 Sense
// Isole la cause de l'echec de l'IMU, SANS passer par
// la librairie LSM6DS3 (pour eliminer les faux coupables :
// mauvais settings, mauvaise version de lib, etc.)
//
// Ce que ce sketch fait :
//  1. Scanne le bus Wire1 (bus interne, ou est cable l'IMU)
//     et affiche toutes les adresses qui repondent.
//  2. Lit directement le registre WHO_AM_I (0x0F) aux deux
//     adresses possibles du LSM6DS3 (0x6A et 0x6B).
//     Valeur attendue par le datasheet : 0x6A (=106 decimal).
//
// Lecture des resultats :
//  - Aucune adresse trouvee au scan -> bus I2C mort ou capteur
//    non alimente (probleme materiel/pin d'activation).
//  - Adresse trouvee mais WHO_AM_I different de 0x6A -> mauvais
//    peripherique a cette adresse, ou lecture corrompue.
//  - Adresse 0x6A trouvee ET WHO_AM_I = 0x6A -> le capteur va
//    bien, le probleme vient de la configuration de la librairie
//    LSM6DS3 (myIMU.settings.*) dans le sketch principal.
//
// LED (si tu n'as pas encore installe l'appli terminal serie) :
//  - VERT fixe  = au moins une adresse trouvee au scan
//  - ROUGE fixe = scan termine, AUCUNE adresse trouvee
//  - BLEU : clignote le WHO_AM_I lu a 0x6A, en deux series
//    separees par une pause (chiffre haut puis chiffre bas,
//    en base 16). Ex : WHO_AM_I=0x6A -> 6 clignotements,
//    pause, 10 clignotements, pause longue, on recommence.
// =================================================

#include <Wire.h>

void setup()
{
  Serial.begin(115200);
  uint32_t depart = millis();
  while (!Serial && (millis() - depart) < 4000) delay(100);
  delay(300);

  pinMode(LED_RED, OUTPUT);
  pinMode(LED_GREEN, OUTPUT);
  pinMode(LED_BLUE, OUTPUT);
  digitalWrite(LED_RED, HIGH);
  digitalWrite(LED_GREEN, HIGH);
  digitalWrite(LED_BLUE, HIGH);

  Serial.println();
  Serial.println("=================================");
  Serial.println("DIAGNOSTIC I2C — Wire1 (bus interne XIAO Sense)");
  Serial.println("=================================");

  Wire1.begin();
  Wire1.setClock(400000);

  // -------- SCAN --------
  Serial.println("Scan des adresses I2C sur Wire1...");

  int trouves = 0;

  for (uint8_t addr = 0x08; addr <= 0x77; addr++)
  {
    Wire1.beginTransmission(addr);
    uint8_t erreur = Wire1.endTransmission();

    if (erreur == 0)
    {
      trouves++;
      Serial.print("  -> Peripherique trouve a l'adresse 0x");
      if (addr < 16) Serial.print("0");
      Serial.println(addr, HEX);
    }
  }

  if (trouves == 0)
  {
    Serial.println("AUCUN peripherique trouve sur Wire1.");
    Serial.println("-> Bus I2C mort, ou capteur non alimente/mal cable.");
    digitalWrite(LED_RED, LOW);
  }
  else
  {
    Serial.print(trouves);
    Serial.println(" peripherique(s) trouve(s).");
    digitalWrite(LED_GREEN, LOW);
  }

  // -------- WHO_AM_I aux deux adresses possibles --------
  Serial.println();
  Serial.println("Lecture directe du registre WHO_AM_I (0x0F) :");

  int who6A = lireRegistre(0x6A, 0x0F);
  int who6B = lireRegistre(0x6B, 0x0F);

  Serial.print("  Adresse 0x6A -> WHO_AM_I = ");
  afficherResultat(who6A);

  Serial.print("  Adresse 0x6B -> WHO_AM_I = ");
  afficherResultat(who6B);

  Serial.println();
  Serial.println("Valeur attendue par le datasheet LSM6DS3(TR-C) : 0x6A (106)");
  Serial.println();

  int whoValide = (who6A == 0x6A) ? who6A : ((who6B == 0x6A) ? who6B : who6A);

  if (whoValide < 0)
  {
    Serial.println("CONCLUSION : pas de reponse du tout -> probleme materiel/alimentation.");
  }
  else if (whoValide != 0x6A)
  {
    Serial.println("CONCLUSION : une reponse arrive mais la valeur est fausse -> bus bruite, pull-ups, ou vitesse I2C trop elevee. Essaie Wire1.setClock(100000).");
  }
  else
  {
    Serial.println("CONCLUSION : le capteur repond correctement au niveau materiel.");
    Serial.println("Le probleme est donc dans la configuration de la librairie LSM6DS3 (myIMU.settings.*) du sketch principal, pas dans le materiel.");
  }

  Serial.println("=================================");
}

int lireRegistre(uint8_t adresseI2C, uint8_t registre)
{
  Wire1.beginTransmission(adresseI2C);
  Wire1.write(registre);

  if (Wire1.endTransmission(false) != 0)
  {
    return -1; // pas d'ACK : rien a cette adresse
  }

  if (Wire1.requestFrom(adresseI2C, (uint8_t)1) != 1)
  {
    return -2; // pas de reponse a la lecture
  }

  return Wire1.read();
}

void afficherResultat(int valeur)
{
  if (valeur == -1)
  {
    Serial.println("(pas de reponse / pas d'ACK)");
  }
  else if (valeur == -2)
  {
    Serial.println("(ACK ok mais pas de donnee lue)");
  }
  else
  {
    Serial.print("0x");
    if (valeur < 16) Serial.print("0");
    Serial.print(valeur, HEX);
    Serial.print(" (");
    Serial.print(valeur);
    Serial.println(")");
  }
}

void loop()
{
  // Rien : tout se joue dans setup(). On garde loop() vide
  // pour que le resultat reste affiche/clignotant en continu.
}
