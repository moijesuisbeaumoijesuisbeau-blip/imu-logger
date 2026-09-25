// =================================================
// DIAGNOSTIC MINIMAL I2C — XIAO nRF52840 Sense
// V2 : teste Wire ET Wire1, pour determiner lequel des
// deux bus porte reellement l'IMU embarque (ca depend de
// la version du core Seeeduino installee).
//
// LED :
//  - VERTE fixe = peripherique trouve sur Wire  (bus externe/D4-D5)
//  - BLEUE fixe = peripherique trouve sur Wire1 (bus interne)
//  - Les deux allumees = trouve sur les deux (rare)
//  - ROUGE fixe = rien trouve nulle part -> probleme d'alimentation
//    du capteur ou de cablage materiel, pas juste le bus.
// =================================================

#include <Wire.h>

bool scanBus(TwoWire &bus, const char *nomBus)
{
  Serial.print("Scan sur ");
  Serial.print(nomBus);
  Serial.println(" ...");

  bus.begin();
  bus.setClock(400000);

  int trouves = 0;

  for (uint8_t addr = 0x08; addr <= 0x77; addr++)
  {
    bus.beginTransmission(addr);
    uint8_t erreur = bus.endTransmission();

    if (erreur == 0)
    {
      trouves++;
      Serial.print("  -> Peripherique trouve sur ");
      Serial.print(nomBus);
      Serial.print(" a l'adresse 0x");
      if (addr < 16) Serial.print("0");
      Serial.println(addr, HEX);
    }
  }

  if (trouves == 0)
  {
    Serial.print("  Rien trouve sur ");
    Serial.println(nomBus);
  }

  return trouves > 0;
}

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
  Serial.println("DIAGNOSTIC I2C — Wire et Wire1");
  Serial.println("=================================");

  bool trouveWire  = scanBus(Wire,  "Wire  (bus externe)");
  bool trouveWire1 = scanBus(Wire1, "Wire1 (bus interne)");

  Serial.println();

  if (!trouveWire && !trouveWire1)
  {
    Serial.println("CONCLUSION : rien trouve sur AUCUN des deux bus.");
    Serial.println("-> Probleme d'alimentation du capteur (pin d'activation manquant),");
    Serial.println("   de soudure, ou ce n'est pas une variante Sense avec IMU embarque.");
    digitalWrite(LED_RED, LOW);
  }
  else
  {
    if (trouveWire)
    {
      Serial.println("CONCLUSION PARTIELLE : le capteur repond sur WIRE (pas Wire1).");
      digitalWrite(LED_GREEN, LOW);
    }
    if (trouveWire1)
    {
      Serial.println("CONCLUSION PARTIELLE : le capteur repond sur WIRE1.");
      digitalWrite(LED_BLUE, LOW);
    }
  }

  Serial.println("=================================");
}

void loop()
{
}
