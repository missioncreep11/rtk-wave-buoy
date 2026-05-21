// menuIMU.ino
// IMU configuration menu function
void menuConfigure_IMU() {
  while (1) {
    Serial.println();
    Serial.println(F("Menu: Configure IMU Sensor"));

    Serial.print(F("1) IMU Logging                : "));
    if (settings.sensor_IMU.log) Serial.println(F("Enabled"));
    else Serial.println(F("Disabled"));

    if (settings.sensor_IMU.log) {
      Serial.print(F("2) Log Accelerometer          : "));
      if (settings.sensor_IMU.enableAccel) Serial.println(F("Enabled"));
      else Serial.println(F("Disabled"));

      Serial.print(F("3) Log Gyroscope              : "));
      if (settings.sensor_IMU.enableGyro) Serial.println(F("Enabled"));
      else Serial.println(F("Disabled"));

      Serial.print(F("4) Log Magnetometer           : "));
      if (settings.sensor_IMU.enableMag) Serial.println(F("Enabled"));
      else Serial.println(F("Disabled"));

      Serial.print(F("5) Log Temperature            : "));
      if (settings.sensor_IMU.enableTemp) Serial.println(F("Enabled"));
      else Serial.println(F("Disabled"));

      Serial.print(F("6) IMU Log Rate (ms)          : "));
      Serial.println(settings.sensor_IMU.logRateMs);
    }

    Serial.println(F("x) Exit"));

    byte incoming = getByteChoice(menuTimeout);

    if (incoming == '1') {
      settings.sensor_IMU.log ^= 1;
      if (settings.sensor_IMU.log) {
        beginIMU();
      } else {
        disableIMU();
      }
    }
    else if (settings.sensor_IMU.log && online.imu) {
      if (incoming == '2')
        settings.sensor_IMU.enableAccel ^= 1;
      else if (incoming == '3')
        settings.sensor_IMU.enableGyro ^= 1;
      else if (incoming == '4')
        settings.sensor_IMU.enableMag ^= 1;
      else if (incoming == '5')
        settings.sensor_IMU.enableTemp ^= 1;
      else if (incoming == '6') {
        Serial.println(F("Enter IMU log rate in milliseconds (10-5000):"));
        int newRate = getNumber(menuTimeout);
        if (newRate >= 10 && newRate <= 5000) {
          settings.sensor_IMU.logRateMs = newRate;
        } else {
          Serial.println(F("Invalid rate. Using default."));
        }
      }
      else if (incoming == 'x')
        break;
      else if (incoming == STATUS_GETBYTE_TIMEOUT)
        break;
      else
        printUnknown(incoming);
    }
    else if (incoming == 'x')
      break;
    else if (incoming == STATUS_GETBYTE_TIMEOUT)
      break;
    else
      printUnknown(incoming);
  }
}