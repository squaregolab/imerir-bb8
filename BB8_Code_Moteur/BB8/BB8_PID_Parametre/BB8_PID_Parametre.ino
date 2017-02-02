/* Serial Loop */

#include <SoftwareSerial.h>
#include "Wire.h"
#include "MPU6050_6Axis_MotionApps20.h"
#include "MPU6050.h" // not necessary if using MotionApps include file

#define RXPIN 0
#define TXPIN 1
#define OUTPUT_READABLE_YAWPITCHROLL
#define INTERRUPT_PIN 2
//#define DEBUG

MPU6050 mpu;


// MPU control/status vars
bool dmpReady = false;  // set true if DMP init was successful
uint8_t mpuIntStatus;   // holds actual interrupt status byte from MPU
uint8_t devStatus;      // return status after each device operation (0 = success, !0 = error)
uint16_t packetSize;    // expected DMP packet size (default is 42 bytes)
uint16_t fifoCount;     // count of all bytes currently in FIFO
uint8_t fifoBuffer[64]; // FIFO storage buffer

            // orientation/motion vars
Quaternion q;           // [w, x, y, z]         quaternion container
VectorInt16 aa;         // [x, y, z]            accel sensor measurements
VectorInt16 aaReal;     // [x, y, z]            gravity-free accel sensor measurements
VectorInt16 aaWorld;    // [x, y, z]            world-frame accel sensor measurements
VectorFloat gravity;    // [x, y, z]            gravity vector
float euler[3];         // [psi, theta, phi]    Euler angle container
float ypr[3];           // [yaw, pitch, roll]   yaw/pitch/roll container and gravity vector

            // packet structure for InvenSense teapot demo
uint8_t teapotPacket[14] = { '$', 0x02, 0,0, 0,0, 0,0, 0,0, 0x00, 0x00, '\r', '\n' };



/**
* \author BOUVIER Thibault, GEORGET Marvin, Aurelien WILLAUMEZ
* \version V1.0
* \date 8 d�cembre 2016
* \brief Code pour le control des moteurs grace a l'appli mobile arduino bluetooth
*
*/

int HORIZONTALE = 5;
int VERTICALE = 6;
int DIAGONALE = 4;
int ana_a = 3;// initalisation des voie moteurs
int ana_b = 11;
int MarcheAvantA = 12;
int MarcheAvantB = 13;
float DeltaMin = -11;
float DeltaMax = 4;
float Kprop = 6.5;
float Kint = 0.00045;
float Kderiv = 60.0;
float Prev_err = 0;
float integ = 0;
float Erreur;


SoftwareSerial mySerial(RXPIN, TXPIN); // RX, TX
char commande; // pour lire les commandes envoy� par le module bluetooth

  // ================================================================
  // ===               INTERRUPT DETECTION ROUTINE                ===
  // ================================================================

volatile bool mpuInterrupt = false;     // indicates whether MPU interrupt pin has gone high
void dmpDataReady() {
  mpuInterrupt = true;
}


void setup() {
/* BLUETHOOTH */
  Serial.begin(9600);
  Serial.println("AT");

  mySerial.begin(38400);
  mySerial.println("AT");

  pinMode(MarcheAvantB, OUTPUT);
  pinMode(MarcheAvantA, OUTPUT);
  pinMode(ana_a, OUTPUT);
  pinMode(ana_b, OUTPUT);
  pinMode(HORIZONTALE, OUTPUT);
  pinMode(VERTICALE, OUTPUT);
  pinMode(DIAGONALE, OUTPUT);
  digitalWrite(VERTICALE, LOW);
  digitalWrite(HORIZONTALE, LOW);
  digitalWrite(DIAGONALE, LOW);
/* FIN BLUYETHOOTH*/

  // join I2C bus (I2Cdev library doesn't do this automatically)
#if I2CDEV_IMPLEMENTATION == I2CDEV_ARDUINO_WIRE
  Wire.begin();
  Wire.setClock(400000); // 400kHz I2C clock. Comment this line if having compilation difficulties
#elif I2CDEV_IMPLEMENTATION == I2CDEV_BUILTIN_FASTWIRE
  Fastwire::setup(400, true);
#endif

  // initialize serial communication
  // (115200 chosen because it is required for Teapot Demo output, but it's
  // really up to you depending on your project)
//  Serial.begin(115200);
//  while (!Serial); // wait for Leonardo enumeration, others continue immediately

           // NOTE: 8MHz or slower host processors, like the Teensy @ 3.3v or Ardunio
           // Pro Mini running at 3.3v, cannot handle this baud rate reliably due to
           // the baud timing being too misaligned with processor ticks. You must use
           // 38400 or slower in these cases, or use some kind of external separate
           // crystal solution for the UART timer.

           // initialize device
  Serial.println(F("Initializing I2C devices..."));
  mpu.initialize();
  pinMode(INTERRUPT_PIN, INPUT);

  // verify connection
  Serial.println(F("Testing device connections..."));
  Serial.println(mpu.testConnection() ? F("MPU6050 connection successful") : F("MPU6050 connection failed"));

  // wait for ready
  Serial.println(F("\nSend any character to begin DMP programming and demo: "));
  while (Serial.available() && Serial.read()); // empty buffer
  while (!Serial.available());                 // wait for data
  while (Serial.available() && Serial.read()); // empty buffer again

                         // load and configure the DMP
  Serial.println(F("Initializing DMP..."));
  devStatus = mpu.dmpInitialize();

  // supply your own gyro offsets here, scaled for min sensitivity
  mpu.setXGyroOffset(220);
  mpu.setYGyroOffset(76);
  mpu.setZGyroOffset(-85);
  mpu.setZAccelOffset(1788); // 1688 factory default for my test chip

                 // make sure it worked (returns 0 if so)
  if (devStatus == 0) {
    // turn on the DMP, now that it's ready
    Serial.println(F("Enabling DMP..."));
    mpu.setDMPEnabled(true);

    // enable Arduino interrupt detection
    Serial.println(F("Enabling interrupt detection (Arduino external interrupt 0)..."));
    attachInterrupt(digitalPinToInterrupt(INTERRUPT_PIN), dmpDataReady, RISING);
    mpuIntStatus = mpu.getIntStatus();

    // set our DMP Ready flag so the main loop() function knows it's okay to use it
    Serial.println(F("DMP ready! Waiting for first interrupt..."));
    dmpReady = true;

    // get expected DMP packet size for later comparison
    packetSize = mpu.dmpGetFIFOPacketSize();
  }
  else {
    // ERROR!
    // 1 = initial memory load failed
    // 2 = DMP configuration updates failed
    // (if it's going to break, usually the code will be 1)
    Serial.print(F("DMP Initialization failed (code "));
    Serial.print(devStatus);
    Serial.println(F(")"));
  }
}

float PID(float error){
    float sortie;
    if(error < DeltaMin || error > DeltaMax){
          integ += Kint * error;
          sortie = integ + Kprop * error + Kderiv * (error-Prev_err);
          Prev_err = error;
    }
    return sortie;  
}

bool SensDeRotation(float error){
    bool Sens_De_Rotation = LOW;
    if(error < DeltaMin){
        Sens_De_Rotation = HIGH;
    }
    else if(error > DeltaMax){
        Sens_De_Rotation = LOW;
    }
    return Sens_De_Rotation;
}

void loop() {

  // if programming failed, don't try to do anything
  if (!dmpReady) return;

   float Consigne = (DeltaMin + DeltaMax) / 2;
   float mesure = ypr[1] * 180/M_PI;
   //Serial.println(mesure);
   float erreur = Consigne - mesure;
   //Serial.print("erreur :");
   Serial.println(erreur);
   
  while (Serial.available()) {
    commande = Serial.read();
    Serial.print(commande); //echo
    mySerial.print(commande);
    Serial.print("\n");

    switch (commande)
    {
    case 'F':
      digitalWrite(MarcheAvantA, LOW);
      digitalWrite(MarcheAvantB, LOW);
      digitalWrite(VERTICALE, HIGH);
      digitalWrite(DIAGONALE, LOW);
      digitalWrite(HORIZONTALE, LOW);
      analogWrite(ana_a, 250);// donne une valeur de vitesse au moteur 80 demarrage 250 valeur max
      analogWrite(ana_b, 250);
      break;

    case 'B':
      digitalWrite(MarcheAvantA, HIGH);
      digitalWrite(MarcheAvantB, HIGH);
      digitalWrite(VERTICALE, HIGH);
      digitalWrite(DIAGONALE, LOW);
      digitalWrite(HORIZONTALE, LOW);
      analogWrite(ana_a, 250);// donne une valeur de vitesse au moteur 80 demarrage 250 valeur max
      analogWrite(ana_b, 250);
      break;

    case 'L':
      digitalWrite(MarcheAvantA, HIGH);
      digitalWrite(MarcheAvantB, LOW);
      digitalWrite(VERTICALE, LOW);
      digitalWrite(DIAGONALE, LOW);
      digitalWrite(HORIZONTALE, HIGH);
      analogWrite(ana_a, 250);// donne une valeur de vitesse au moteur 80 demarrage 250 valeur max
      analogWrite(ana_b, 250);
      break;

    case 'R':
      digitalWrite(MarcheAvantB, HIGH);
      digitalWrite(MarcheAvantA, LOW);
      digitalWrite(VERTICALE, LOW);
      digitalWrite(DIAGONALE, LOW);
      digitalWrite(HORIZONTALE, HIGH);
      analogWrite(ana_a, 250);// donne une valeur de vitesse au moteur 80 demarrage 250 valeur max
      analogWrite(ana_b, 250);
      break;

    case 'G':
      digitalWrite(VERTICALE, LOW);
      digitalWrite(DIAGONALE, HIGH);
      digitalWrite(HORIZONTALE, LOW);
      analogWrite(ana_a, 250);// donne une valeur de vitesse au moteur 80 demarrage 250 valeur max
      analogWrite(ana_b, 70);
      break;

    case 'I':
      digitalWrite(VERTICALE, LOW);
      digitalWrite(DIAGONALE, HIGH);
      digitalWrite(HORIZONTALE, LOW);
      analogWrite(ana_a, 70);// donne une valeur de vitesse au moteur 80 demarrage 250 valeur max
      analogWrite(ana_b, 250);
      break;

    case 'H':
      digitalWrite(MarcheAvantA, HIGH);
      digitalWrite(MarcheAvantB, HIGH);
      digitalWrite(VERTICALE, LOW);
      digitalWrite(DIAGONALE, HIGH);
      digitalWrite(HORIZONTALE, LOW);
      analogWrite(ana_a, 125);// donne une valeur de vitesse au moteur 80 demarrage 250 valeur max
      analogWrite(ana_b, 250);
      break;

    case 'J':
      digitalWrite(MarcheAvantA, HIGH);
      digitalWrite(MarcheAvantB, HIGH);
      digitalWrite(VERTICALE, LOW);
      digitalWrite(DIAGONALE, HIGH);
      digitalWrite(HORIZONTALE, LOW);
      analogWrite(ana_a, 250);// donne une valeur de vitesse au moteur 80 demarrage 250 valeur max
      analogWrite(ana_b, 125);
      break;

    case 'S':
      digitalWrite(MarcheAvantA, LOW);
      digitalWrite(MarcheAvantB, LOW);
      digitalWrite(VERTICALE, HIGH);
      digitalWrite(DIAGONALE, LOW);
      digitalWrite(HORIZONTALE, LOW);
      analogWrite(ana_a, 0);// donne une valeur de vitesse au moteur 80 demarrage 250 valeur max
      analogWrite(ana_b, 0);
      break;
      
    case 'D':
      digitalWrite(MarcheAvantA, SensDeRotation(erreur));
      digitalWrite(MarcheAvantB, SensDeRotation(erreur));
      digitalWrite(VERTICALE, HIGH);
      digitalWrite(DIAGONALE, LOW);
      digitalWrite(HORIZONTALE, LOW);
      float pid =  PID(erreur);
      Serial.print("pid : ");
      Serial.println(pid);
      analogWrite(ana_a, abs(pid));// donne une valeur de vitesse au moteur 80 demarrage 250 valeur max
      analogWrite(ana_b, abs(pid));
      break;
    }
  }


  //Serial.print("mpuInterrupt : "); Serial.println(mpuInterrupt, DEC);
  //Serial.print("fifoCount :"); Serial.println(fifoCount, DEC);
  //Serial.print("packetSize :"); Serial.println(packetSize, DEC);

  // wait for MPU interrupt or extra packet(s) available
  //while (mpuInterrupt && fifoCount < packetSize) {
    // reset interrupt flag and get INT_STATUS byte
    mpuInterrupt = false;
    mpuIntStatus = mpu.getIntStatus();

    // get current FIFO count
    fifoCount = mpu.getFIFOCount();

    // check for overflow (this should never happen unless our code is too inefficient)
    if ((mpuIntStatus & 0x10) || fifoCount == 1024) {
      // reset so we can continue cleanly
      mpu.resetFIFO();
      Serial.println(F("FIFO overflow!"));

      // otherwise, check for DMP data ready interrupt (this should happen frequently)
    }
    else if (mpuIntStatus & 0x02) {
      // wait for correct available data length, should be a VERY short wait
      while (fifoCount < packetSize) fifoCount = mpu.getFIFOCount();

      // read a packet from FIFO
      mpu.getFIFOBytes(fifoBuffer, packetSize);

      // track FIFO count here in case there is > 1 packet available
      // (this lets us immediately read more without waiting for an interrupt)
      fifoCount -= packetSize;

#ifdef OUTPUT_READABLE_QUATERNION
      // display quaternion values in easy matrix form: w x y z
      mpu.dmpGetQuaternion(&q, fifoBuffer);
      #ifdef DEBUG
      Serial.print("quat\t");
      Serial.print(q.w);
      Serial.print("\t");
      Serial.print(q.x);
      Serial.print("\t");
      Serial.print(q.y);
      Serial.print("\t");
      Serial.println(q.z);
      #endif
#endif

#ifdef OUTPUT_READABLE_EULER
      // display Euler angles in degrees
      mpu.dmpGetQuaternion(&q, fifoBuffer);
      mpu.dmpGetEuler(euler, &q);
      #ifdef DEBUG
      Serial.print("euler\t");
      Serial.print(euler[0] * 180 / M_PI);
      Serial.print("\t");
      Serial.print(euler[1] * 180 / M_PI);
      Serial.print("\t");
      Serial.println(euler[2] * 180 / M_PI);
      #endif
#endif

#ifdef OUTPUT_READABLE_YAWPITCHROLL
      // display Euler angles in degrees
      mpu.dmpGetQuaternion(&q, fifoBuffer);
      mpu.dmpGetGravity(&gravity, &q);
      mpu.dmpGetYawPitchRoll(ypr, &q, &gravity);
      #ifdef DEBUG
      Serial.print("ypr\t");
      Serial.print(ypr[0] * 180 / M_PI);
      Serial.print("\t");
      Serial.print(ypr[1] * 180 / M_PI);
      Serial.print("\t");
      Serial.println(ypr[2] * 180 / M_PI);
      #endif
#endif

#ifdef OUTPUT_READABLE_REALACCEL
      // display real acceleration, adjusted to remove gravity
      mpu.dmpGetQuaternion(&q, fifoBuffer);
      mpu.dmpGetAccel(&aa, fifoBuffer);
      mpu.dmpGetGravity(&gravity, &q);
      mpu.dmpGetLinearAccel(&aaReal, &aa, &gravity);
      #ifdef DEBUG
      Serial.print("areal\t");
      Serial.print(aaReal.x);
      Serial.print("\t");
      Serial.print(aaReal.y);
      Serial.print("\t");
      Serial.println(aaReal.z);
      #endif
#endif

#ifdef OUTPUT_READABLE_WORLDACCEL
      // display initial world-frame acceleration, adjusted to remove gravity
      // and rotated based on known orientation from quaternion
      mpu.dmpGetQuaternion(&q, fifoBuffer);
      mpu.dmpGetAccel(&aa, fifoBuffer);
      mpu.dmpGetGravity(&gravity, &q);
      mpu.dmpGetLinearAccel(&aaReal, &aa, &gravity);
      mpu.dmpGetLinearAccelInWorld(&aaWorld, &aaReal, &q);
      #ifdef DEBUG
      Serial.print("aworld\t");
      Serial.print(aaWorld.x);
      Serial.print("\t");
      Serial.print(aaWorld.y);
      Serial.print("\t");
      Serial.println(aaWorld.z);
      #endif
#endif

#ifdef OUTPUT_TEAPOT
      // display quaternion values in InvenSense Teapot demo format:
      teapotPacket[2] = fifoBuffer[0];
      teapotPacket[3] = fifoBuffer[1];
      teapotPacket[4] = fifoBuffer[4];
      teapotPacket[5] = fifoBuffer[5];
      teapotPacket[6] = fifoBuffer[8];
      teapotPacket[7] = fifoBuffer[9];
      teapotPacket[8] = fifoBuffer[12];
      teapotPacket[9] = fifoBuffer[13];
      Serial.write(teapotPacket, 14);
      teapotPacket[11]++; // packetCount, loops at 0xFF on purpose
#endif
    }
//  }
}

