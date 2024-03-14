#include <Adafruit_LSM6DS3TRC.h>
Adafruit_LSM6DS3TRC lsm6ds3trc;

#define PI 3.1415926535897932384626433832795
double rotation;
sensors_event_t accel;
sensors_event_t gyro;
sensors_event_t temp;

void setup()
{
    Serial.begin(115200);
    while (!Serial)
        delay(10);

    if (!lsm6ds3trc.begin_I2C())
    {
        Serial.println("Failed to find LSM6DS3TR-C chip");
        while (1)
        {
            delay(10);
        }
    }
    Serial.println("LSM6DS3TR-C Found!");
    lsm6ds3trc.configInt1(false, false, true); // accelerometer DRDY on INT1
    lsm6ds3trc.configInt2(false, true, false); // gyro DRDY on INT2
}

void loop()
{
    updateRotation();
    Serial.println(rotation);

    delay(100);
}

void updateRotation()
{
    lsm6ds3trc.getEvent(&accel, &gyro, &temp);
    static unsigned long timer;
    int interval = 100;
    if (millis() > timer)
    {
        double rotated = (radiansToDegrees(gyro.gyro.z) / 10);
        Serial.println(rotated);
        rotation += rotated;
        timer = millis() + interval;
    }
    if (rotation > 360)
    {
        rotation -= 360;
    }
    if (rotation < 0)
    {
        rotation += 360;
    }
}

double radiansToDegrees(double radians)
{
    return radians * (180 / PI);
}


