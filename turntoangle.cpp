#include <Arduino.h>
#include <Adafruit_NeoPixel.h>
#include <Adafruit_LSM6DS3TRC.h>
#ifdef __AVR__
    #include <avr/power.h>
#endif

double wrapAngle(int rotationToWrap)
{
    if (rotationToWrap > 360)
    {
        rotationToWrap -= 360;
    }
    if (rotationToWrap < 0)
    {
        rotationToWrap += 360;
    }
    return rotationToWrap;
}

void driveRight(int speed) // Define the driveRight function
{
    // Implementation goes here
}

void driveLeft(int speed) // Define the driveLeft function
{
    // Implementation goes here
}

double rotationInDegrees = 0; // Define the rotationInDegrees variable

void turnToAngle(double wantedRotation)
{
    int driveSpeed;

    double rotationLeft = abs(wantedRotation - rotationInDegrees);
    if (rotationLeft < 15)
    {
        driveSpeed = 255 * (rotationLeft / 30);
    }
    else
    {
        driveSpeed = 255;
    }
    if (isRightTurnFaster(wantedRotation))
    {
        driveRight(driveSpeed);
    }
    else
    {
        driveLeft(driveSpeed);
    }
}

bool isRightTurnFaster(double wantedRotation)
{
    double rightDiffrence = wrapAngle(rotationInDegrees - wantedRotation);
    double leftDiffrence = wrapAngle(wantedRotation - rotationInDegrees);
    if (rightDiffrence > leftDiffrence)
    {
        return false;
    }
    else
    {
        return true;
    }
}

double wrapAngle(int rotationToWrap)
{
    if (rotationToWrap > 360)
    {
        rotationToWrap -= 360;
    }
    if (rotationToWrap < 0)
    {
        rotationToWrap += 360;
    }
    return rotationToWrap;
}