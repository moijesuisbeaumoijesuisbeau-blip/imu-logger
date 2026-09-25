#include <Wire.h>

#define PIN_IMU_POWER 15

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
  Serial.println("DIAGNOSTIC I2C — avec activation alimentation IMU (pin 15)");
  Serial.println("=================================");

  // ETAPE CLE : activer l'alimentation de l'IMU AVANT tout
  // le reste (avant Wire.begin()/Wire1.begin(), pour ne pas
  // "phantom power" le capteur via les pull-ups I2C).
  pinMode(PIN_IMU_POWER, OUTPUT);
  digitalWrite(PIN_IMU_POWER, HIGH);
  Serial.println("Pin 15 (alimentation IMU) mis a HIGH.");
  delay(20); // marge par rapport aux ~3ms requis par le regulateur

  bool trouveWire  = scanBus(Wire,  "Wire  (bus externe)");
  bool trouveWire1 = scanBus(Wire1, "Wire1 (bus interne)");

  Serial.println();

  if (!trouveWire && !trouveWire1)
  {
    Serial.println("CONCLUSION : toujours rien, meme avec l'alimentation activee.");
    Serial.println("-> Le pin 15 n'est peut-etre pas le bon sur ta variante exacte, ou souci materiel.");
    digitalWrite(LED_RED, LOW);
  }
  else
  {
    if (trouveWire)
    {
      Serial.println("SUCCES sur WIRE !");
      digitalWrite(LED_GREEN, LOW);
    }
    if (trouveWire1)
    {
      Serial.println("SUCCES sur WIRE1 !");
      digitalWrite(LED_BLUE, LOW);
    }
  }

  Serial.println("=================================");
}

void loop()
{
}
