// Demo: NMEA2000 library. Send main cabin temperature to the bus.

#include <Arduino.h>
#include <NMEA2000_CAN.h>  // This will automatically choose right CAN library and create suitable NMEA2000 object
#include <N2kMessages.h>

#include <Wire.h>
#include <SPI.h>
#include <Adafruit_Sensor.h>
#include <Adafruit_BME280.h>
#include <TinyGPS++.h>
#include <TimeLib.h>

#include "quaternionFilters.h"
#include "MPU9250.h"

#define SEALEVELPRESSURE_HPA (1013.25)

Adafruit_BME280 bme;
MPU9250 myIMU;
TinyGPSPlus gps;
float CurrentAltitude = 0.0;
time_t gpsDate = 0;
unsigned long DaysSince1970;

int myLed  = 13; 
static const uint32_t GPSBaud = 9600;

tN2kMsg N2kMsg;


// List here messages your device will transmit.
const unsigned long TransmitMessages[] PROGMEM={126992L,127250L,127251L,127257L,129025L,129026L,129029L,130311L,0};

void setup() {
  // Set Product information
  NMEA2000.SetProductInformation("00000001", // Manufacturer's Model serial code
                                 100, // Manufacturer's product code
                                 "GPS_AHRS_ENV",  // Manufacturer's Model ID
                                 "1.0.0.0 (2017-06-10)",  // Manufacturer's Software version code
                                 "1.0.0.0 (2017-06-10)" // Manufacturer's Model version
                                 );
  // Set device information
  NMEA2000.SetDeviceInformation(70777, // Unique number. Use e.g. Serial number.
                                140, 
                                60, 
                                2048                              
                               );
  // Uncomment 2 rows below to see, what device will send to bus. Use e.g. OpenSkipper or Actisense NMEA Reader                           
  Serial.begin(115200);
  NMEA2000.SetForwardStream(&Serial);
  // If you want to use simple ascii monitor like Arduino Serial Monitor, uncomment next line
  NMEA2000.SetForwardType(tNMEA2000::fwdt_Text); // Show in clear text. Leave uncommented for default Actisense format.

  // If you also want to see all traffic on the bus use N2km_ListenAndNode instead of N2km_NodeOnly below
  NMEA2000.SetMode(tNMEA2000::N2km_NodeOnly,44);
  //NMEA2000.SetDebugMode(tNMEA2000::dm_Actisense); // Uncomment this, so you can test code without CAN bus chips on Arduino Mega
  NMEA2000.EnableForward(false); // Disable all msg forwarding to USB (=Serial)
  //Here we tell library, which PGNs we transmit
  NMEA2000.ExtendTransmitMessages(TransmitMessages);
  NMEA2000.Open();

  bool status;
    // default settings
  status = bme.begin();
  if (!status) {
        Serial.println("Could not find a valid BME280 sensor, check wiring!");
        while (1);
  }
  Serial.println("-- BME Begin --");
  delay(100); // let sensor boot up

  bme.setSampling(Adafruit_BME280::MODE_NORMAL,
                  Adafruit_BME280::SAMPLING_X8, // temperature
                  Adafruit_BME280::SAMPLING_X16, // pressure
                  Adafruit_BME280::SAMPLING_X8, // humidity
                  Adafruit_BME280::FILTER_OFF   );


  byte c = myIMU.readByte(MPU9250_ADDRESS, WHO_AM_I_MPU9250);
  Serial.print("MPU9250 "); Serial.print("I AM "); Serial.print(c, HEX);
  Serial.print(" I should be "); Serial.println(0x71, HEX);
  if (c == 0x71) // WHO_AM_I should always be 0x68
  {
    Serial.println("MPU9250 is online...");

    // Start by performing self test and reporting values
    myIMU.MPU9250SelfTest(myIMU.SelfTest);
    Serial.print("x-axis self test: acceleration trim within : ");
    Serial.print(myIMU.SelfTest[0],1); Serial.println("% of factory value");
    Serial.print("y-axis self test: acceleration trim within : ");
    Serial.print(myIMU.SelfTest[1],1); Serial.println("% of factory value");
    Serial.print("z-axis self test: acceleration trim within : ");
    Serial.print(myIMU.SelfTest[2],1); Serial.println("% of factory value");
    Serial.print("x-axis self test: gyration trim within : ");
    Serial.print(myIMU.SelfTest[3],1); Serial.println("% of factory value");
    Serial.print("y-axis self test: gyration trim within : ");
    Serial.print(myIMU.SelfTest[4],1); Serial.println("% of factory value");
    Serial.print("z-axis self test: gyration trim within : ");
    Serial.print(myIMU.SelfTest[5],1); Serial.println("% of factory value");

    // Calibrate gyro and accelerometers, load biases in bias registers
    myIMU.calibrateMPU9250(myIMU.gyroBias, myIMU.accelBias);
    Serial.print("gyro  biases (mg)"); Serial.print(1000.*myIMU.gyroBias[0]); Serial.print(1000.*myIMU.gyroBias[1]); Serial.println(1000.*myIMU.gyroBias[2]);
    Serial.print("accel biases (mg)"); Serial.print(1000.*myIMU.accelBias[0]); Serial.print(1000.*myIMU.accelBias[1]); Serial.println(1000.*myIMU.accelBias[2]);
    myIMU.initMPU9250();
    // Initialize device for active mode read of acclerometer, gyroscope, and
    // temperature
    Serial.println("MPU9250 initialized for active data mode....");

    // Read the WHO_AM_I register of the magnetometer, this is a good test of
    // communication
    byte d = myIMU.readByte(AK8963_ADDRESS, WHO_AM_I_AK8963);
    Serial.print("AK8963 "); Serial.print("I AM "); Serial.print(d, HEX);
    Serial.print(" I should be "); Serial.println(0x48, HEX);
    // Get magnetometer calibration from AK8963 ROM
    myIMU.initAK8963(myIMU.magCalibration);
    // Initialize device for active mode read of magnetometer
    Serial.println("AK8963 initialized for active data mode....");
  } // if (c == 0x71)
  else
  {
    Serial.print("Could not connect to MPU9250: 0x");
    Serial.println(c, HEX);
    while(1) ; // Loop forever if communication doesn't happen
  }

  Serial1.begin(GPSBaud);
  
  pinMode(myLed, OUTPUT);
  digitalWrite(myLed, HIGH);

}

double ReadCabinTemp() {
//  myIMU.tempCount = myIMU.readTempData();
//  myIMU.temperature = ((float) myIMU.tempCount) / 333.87 + 21.0 - 4.0;
  double temp = bme.readTemperature() - 2.0;
// à 28, lit 30
//  Serial.print("Temp bme: ");
//  Serial.print(temp);
//  Serial.print(",   Temp 9250: ");
//  Serial.println(myIMU.temperature);
  return CToKelvin(temp); 
}

double ReadPress() {
  double press = bme.seaLevelForAltitude(CurrentAltitude, bme.readPressure()) / 100.0F;
//  Serial.print("Pression :");
//  Serial.println(press);
  return press - 3.0F; 
}

double ReadHumidity() {
  double hum = bme.readHumidity() ;
//  Serial.print("humidity :");
//  Serial.println(hum);
  return hum; 
}

double ReadWaterTemp() {
  myIMU.tempCount = myIMU.readTempData() / 100.0F;  // Read the adc values
  // Temperature in degrees Centigrade
  myIMU.temperature = ((float) myIMU.tempCount) / 333.87 + 21.0;
//  Serial.print("Water TEMP :");
//  Serial.println(myIMU.temperature);
  return CToKelvin(myIMU.temperature); // Read here the true temperature e.g. from analog input
}

#define TempUpdatePeriod 2000

void SendN2kTemperature() {
  static unsigned long TempUpdated=millis();

  if ( TempUpdated+TempUpdatePeriod<millis() ) {
    digitalWrite(myLed, !digitalRead(myLed));
    TempUpdated=millis();
    SetN2kEnvironmentalParameters(N2kMsg, 1, N2kts_InsideTemperature, ReadCabinTemp(), N2khs_InsideHumidity, ReadHumidity(), ReadPress() );
    NMEA2000.SendMsg(N2kMsg);
//    Serial.print(millis()); Serial.println(", Temperature send ready");
  }
}

void UpdateAHRS() {
  if (myIMU.readByte(MPU9250_ADDRESS, INT_STATUS) & 0x01)
  {  
    myIMU.readAccelData(myIMU.accelCount);  // Read the x/y/z adc values
    myIMU.getAres();

    // Now we'll calculate the accleration value into actual g's
    // This depends on scale being set
    myIMU.ax = (float)myIMU.accelCount[0]*myIMU.aRes; // - accelBias[0];
    myIMU.ay = (float)myIMU.accelCount[1]*myIMU.aRes; // - accelBias[1];
    myIMU.az = (float)myIMU.accelCount[2]*myIMU.aRes; // - accelBias[2];

    myIMU.readGyroData(myIMU.gyroCount);  // Read the x/y/z adc values
    myIMU.getGres();

    // Calculate the gyro value into actual degrees per second
    // This depends on scale being set
    myIMU.gx = (float)myIMU.gyroCount[0]*myIMU.gRes;
    myIMU.gy = (float)myIMU.gyroCount[1]*myIMU.gRes;
    myIMU.gz = (float)myIMU.gyroCount[2]*myIMU.gRes;

    myIMU.readMagData(myIMU.magCount);  // Read the x/y/z adc values
    myIMU.getMres();
    // User environmental x-axis correction in milliGauss, should be
    // automatically calculated
    myIMU.magbias[0] = +114.;
    // User environmental x-axis correction in milliGauss TODO axis??
    myIMU.magbias[1] = +354.;
    // User environmental x-axis correction in milliGauss
    myIMU.magbias[2] = -112.;

    // Calculate the magnetometer values in milliGauss
    // Include factory calibration per data sheet and user environmental
    // corrections
    // Get actual magnetometer value, this depends on scale being set
    myIMU.mx = (float)myIMU.magCount[0]*myIMU.mRes*myIMU.magCalibration[0] -
               myIMU.magbias[0];
    myIMU.my = (float)myIMU.magCount[1]*myIMU.mRes*myIMU.magCalibration[1] -
               myIMU.magbias[1];
    myIMU.mz = (float)myIMU.magCount[2]*myIMU.mRes*myIMU.magCalibration[2] -
               myIMU.magbias[2];
  } // if (readByte(MPU9250_ADDRESS, INT_STATUS) & 0x01)

  // Must be called before updating quaternions!
  myIMU.updateTime();

  // Sensors x (y)-axis of the accelerometer is aligned with the y (x)-axis of
  // the magnetometer; the magnetometer z-axis (+ down) is opposite to z-axis
  // (+ up) of accelerometer and gyro! We have to make some allowance for this
  // orientationmismatch in feeding the output to the quaternion filter. For the
  // MPU-9250, we have chosen a magnetic rotation that keeps the sensor forward
  // along the x-axis just like in the LSM9DS0 sensor. This rotation can be
  // modified to allow any convenient orientation convention. This is ok by
  // aircraft orientation standards! Pass gyro rate as rad/s
  //  MadgwickQuaternionUpdate(ax, ay, az, gx*PI/180.0f, gy*PI/180.0f, gz*PI/180.0f,  my,  mx, mz);
  MahonyQuaternionUpdate(-myIMU.ax, myIMU.ay, myIMU.az, myIMU.gx*DEG_TO_RAD,
                         -myIMU.gy*DEG_TO_RAD, -myIMU.gz*DEG_TO_RAD, myIMU.my,
                         -myIMU.mx, myIMU.mz, myIMU.deltat);

  myIMU.delt_t = millis() - myIMU.count;
  if (myIMU.delt_t > 500) {
      myIMU.yaw   = atan2(2.0f * (*(getQ()+1) * *(getQ()+2) + *getQ() *
                    *(getQ()+3)), *getQ() * *getQ() + *(getQ()+1) * *(getQ()+1)
                    - *(getQ()+2) * *(getQ()+2) - *(getQ()+3) * *(getQ()+3));
      myIMU.pitch = -asin(2.0f * (*(getQ()+1) * *(getQ()+3) - *getQ() *
                    *(getQ()+2)));
      myIMU.roll  = atan2(2.0f * (*getQ() * *(getQ()+1) + *(getQ()+2) *
                    *(getQ()+3)), *getQ() * *getQ() - *(getQ()+1) * *(getQ()+1)
                    - *(getQ()+2) * *(getQ()+2) + *(getQ()+3) * *(getQ()+3));
      // myIMU.pitch *= RAD_TO_DEG;
      // myIMU.yaw   *= RAD_TO_DEG;
      // Declination of SparkFun Electronics (40°05'26.6"N 105°11'05.9"W) is
      //   8° 30' E  ± 0° 21' (or 8.5°) on 2016-07-19
      // - http://www.ngdc.noaa.gov/geomag-web/#declination
      myIMU.yaw   += (0.6 * DEG_TO_RAD);;
      // myIMU.roll  *= RAD_TO_DEG;

      //  Serial.print("Yaw, Pitch, Roll: ");
      //  Serial.print(myIMU.yaw * RAD_TO_DEG, 2);
      //  Serial.print(", ");
      //  Serial.print(myIMU.pitch * RAD_TO_DEG, 2);
      //  Serial.print(", ");
      //  Serial.println(myIMU.roll * RAD_TO_DEG, 2);

      //  Serial.print("rate = ");
      //  Serial.print((float)myIMU.sumCount/myIMU.sum, 2);
      //  Serial.println(" Hz");

      SetN2kAttitude(N2kMsg, 1, myIMU.yaw, myIMU.pitch, myIMU.roll);
      NMEA2000.SendMsg(N2kMsg);

      SetN2kRateOfTurn(N2kMsg, 1, myIMU.gz * DEG_TO_RAD);
      NMEA2000.SendMsg(N2kMsg);

      SetN2kMagneticHeading(N2kMsg, 1, myIMU.yaw);
      NMEA2000.SendMsg(N2kMsg);

      myIMU.count = millis();
      myIMU.sumCount = 0;
      myIMU.sum = 0;
  } // if (myIMU.delt_t > 500)
}

void UpdateGPS() {
  TimeElements tm;

  while (Serial1.available())
    if (gps.encode(Serial1.read())) //Serial.print("Phrase NMEA0183 reconnue"); 

  if (gps.altitude.isValid() and gps.altitude.isUpdated()) CurrentAltitude = gps.altitude.meters();

  if (gps.date.isUpdated())
      if(gps.date.isValid() and gps.time.isValid()) {
        tm.Second = gps.time.second();
        tm.Minute = gps.time.minute();
        tm.Hour = gps.time.hour();
        tm.Day = gps.date.day();
        tm.Month = gps.date.month();
        tm.Year = gps.date.year() - 1970;
        gpsDate = makeTime(tm);
        DaysSince1970 = gpsDate / 86400;
        Serial.print(" Year :"); Serial.print(tm.Year);
        Serial.print(" gpsDate :"); Serial.print(gpsDate);
        Serial.print(" days :"); Serial.println(DaysSince1970);

	      SetN2kSystemTime(N2kMsg, 1, DaysSince1970, fmod(gpsDate, 86400) + (gps.time.centisecond() / 100.0F));
        NMEA2000.SendMsg(N2kMsg);
      }

  if (gps.location.isUpdated()) 
      if (gps.location.isValid()) {
	SetN2kLatLonRapid(N2kMsg, gps.location.lat(), gps.location.lng());
        NMEA2000.SendMsg(N2kMsg);
  }

  if (gps.hdop.isUpdated())
      if (gps.date.isValid() and gps.time.isValid() and gps.location.isValid() and gps.altitude.isValid() 
          and gps.satellites.isValid() and gps.satellites.value() and gps.hdop.isValid()) {
 //       Serial.print("days :"); Serial.println(DaysSince1970); Serial.print("  "); Serial.print(fmod(gpsDate, 3600)); Serial.print("  "); Serial.println(gps.time.centisecond());
        SetN2kPGN129029(N2kMsg, 1, DaysSince1970, fmod(gpsDate, 86400) + (gps.time.centisecond() / 100.0F), gps.location.lat(),
		    gps.location.lng(), CurrentAltitude, N2kGNSSt_GPS, N2kGNSSm_GNSSfix, gps.satellites.value(), gps.hdop.value());
        NMEA2000.SendMsg(N2kMsg);
      }

  if (gps.speed.isUpdated() and gps.course.isUpdated())
      if (gps.speed.isValid() and gps.course.isValid()) {
	      SetN2kCOGSOGRapid(N2kMsg, 1, N2khr_true, gps.course.deg() * DEG_TO_RAD, gps.speed.mps());
        NMEA2000.SendMsg(N2kMsg);
      }

}

void loop() {
  SendN2kTemperature();
  UpdateAHRS();
  UpdateGPS();
  NMEA2000.ParseMessages();
}
