/* 
  azimuth.ino - bussola eletronica implementada com chip HMC5883 (magnetometro 3 eixos)
  que fornece o azimute verdadeiro a cada 1s, alem dos dados do sensor
  
  O programa utiliza as bibliotecas I2Cdev e HMC5883L, disponiveis em https://github.com/jrowberg/i2cdevlib
  
  O modulo sensor se conecta via porta I2C do Arduino e os dados sao enviados atraves da porta serial
  (taxa de 9600bps).
  
*/

#include "Wire.h"
#include "I2Cdev.h"
#include "HMC5883L.h"

HMC5883L mag;
int16_t mx, my, mz;

// parametros da calibracao (determinado pelo pos-processamento)
const double X_OFFSET = 55.0;
const double Y_OFFSET = -109.5;
const double Z_OFFSET = -17.0;

//declinacao magnetica
const double DECLINATION = -22.11; //camaragibe-aldeia

#define LED_PIN 13
bool blinkState = false;

void setup() {
    Wire.begin();
    Serial.begin(9600);

    // initialize device
    Serial.println("Initializing I2C devices...");
    mag.initialize();

    // verify connection
    Serial.println("Testing device connections...");
    Serial.println(mag.testConnection() ? "HMC5883L connection successful" : "HMC5883L connection failed");

    // configure Arduino LED for
    pinMode(LED_PIN, OUTPUT);
    
    //imprimir dados do processamento
    Serial.print("OFFSETS (em x,y e z): ");
    Serial.print(X_OFFSET);
    Serial.print(",");
    Serial.print(Y_OFFSET);
    Serial.print(",");
    Serial.println(Z_OFFSET);
    Serial.print("DECLINACAO MAGNETICA: ");
    Serial.print(DECLINATION);
    Serial.println("(Camaragibe/PE)");
    
    //
    Serial.println("SENSOR_VALUES (raw,corrected and azimuth):");
}

void loop() {
  delay(1000);
  
  // read raw heading measurements from device
  mag.getHeading(&mx, &my, &mz);
    
  //correct values
  double mag_x,mag_y,mag_z;
  mag_x = (double) mx - X_OFFSET;
  mag_y = (double) my - Y_OFFSET;
  mag_z = (double) mz - Z_OFFSET;
    
  // display tab-separated mag x/y/z values
  Serial.print("values=");Serial.print("\t");
  Serial.print((double)mx); Serial.print("\t");
  Serial.print((double)my); Serial.print("\t");
  Serial.print((double)mz); Serial.print("\t");
  Serial.print(mag_x); Serial.print("\t");
  Serial.print(mag_y); Serial.print("\t");
  Serial.print(mag_z); Serial.print("\t");
    
  // calculo do azimute em graus. 0 degree indicates North
    
    double heading = (180/M_PI) * atan2(mag_y, mag_x) + DECLINATION;
    if(heading < 0)
      heading += 360;
    
    Serial.println(heading);
   
    // blink LED to indicate activity
    blinkState = !blinkState;
    digitalWrite(LED_PIN, blinkState);
}
