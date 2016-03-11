#include <CurieBLE.h>

#define ledPin 13

BLEPeripheral blePeripheral;
BLEService firstService("19B10000-E8F2-537E-4F6C-D104768A1214");
BLEUnsignedCharCharacteristic switchCharacteristic("19B10001-E8F2-537E-4F6C-D104768A1214", BLERead | BLEWrite);

void setup() {
  // put your setup code here, to run once:
  Serial.begin(9600);
  pinMode(ledPin, OUTPUT);
  blePeripheral.setLocalName("test");
  blePeripheral.setAdvertisedServiceUuid(firstService.uuid());

  blePeripheral.addAttribute(firstService);
  blePeripheral.addAttribute(switchCharacteristic);

  switchCharacteristic.setValue(0);

  blePeripheral.begin();


}

void loop() {
  // put your main code here, to run repeatedly:
  BLECentral central = blePeripheral.central();

  if (central) {
    Serial.print("Connected to: ");
    Serial.println(central.address());

    if (switchCharacteristic.written()) {
      if (switchCharacteristic.value()) {   // any value other than 0
        Serial.println("LED on");
        digitalWrite(ledPin, HIGH);         // will turn the LED on
      } else {                              // a 0 value
        Serial.println(F("LED off"));
        digitalWrite(ledPin, LOW);          // will turn the LED off
      }
    }
  }
}
