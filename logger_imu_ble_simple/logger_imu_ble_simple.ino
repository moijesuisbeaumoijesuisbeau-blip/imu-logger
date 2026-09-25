#include "LSM6DS3.h"
#include "Wire.h"

LSM6DS3 myIMU(I2C_MODE, 0x6A);

void setup()
{
  Serial.begin(115200);
  uint32_t depart = millis();
  while (!Serial && (millis() - depart) < 4000) delay(100);
  delay(300);

  pinMode(LED_RED, OUTPUT);
  pinMode(LED_GREEN, OUTPUT);
  digitalWrite(LED_RED, HIGH);
  digitalWrite(LED_GREEN, HIGH);

  Serial.println();
  Serial.println("=================================");
  Serial.println("TEST minimal, config identique a l'exemple officiel Seeed (Wire, 0x6A)");
  Serial.println("=================================");

  int code = myIMU.begin();

  Serial.print("myIMU.begin() a renvoye : ");
  Serial.println(code);

  if (code == 0)
  {
    Serial.println("SUCCES : l'IMU repond avec la config par defaut.");
    digitalWrite(LED_GREEN, LOW);
  }
  else
  {
    Serial.println("ECHEC meme avec la config par defaut (Wire, 0x6A, sans reglages custom).");
    digitalWrite(LED_RED, LOW);
  }

  Serial.println("=================================");
}

void loop()
{
  if (Serial.available())
  {
    char c = Serial.read();
    if (c == 'r' || c == 'R')
    {
      Serial.print("Accel X = ");
      Serial.println(myIMU.readFloatAccelX());
    }
  }
}
