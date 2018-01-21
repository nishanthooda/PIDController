#include <stdbool.h>
#include "PID.h"

PID::PID() {
    kP = kI = kD = setPoint = 0;
    lastTime = millis();     // THIS COULD CAUSE SOME SERIOUS ISSUES, LOOK THIS UP
}

PID::PID(double kP, double kI, double kD, double setPoint) : kP(kP), kI(kI), kD(kD), setPoint(setPoint) {
    lastTime = millis();      // SEE ABOVE MESSAGE IN DEFAULT CONSTRUCTOR
}

void PID::SetTunings(double kP, double kI, double kD, int pOn) {
    if (kP < 0 || kI < 0 || kD < 0) return;

    pOnE = pOn == P_ON_E;

    double sampleTimeInSec = ((double)sampleTime) / 1000;
    this->kP = kP;
    this->kI = kI * sampleTimeInSec;
    this->kD = kD / sampleTimeInSec;

    if (controllerDirection == REVERSE) {
        this->kP = (0-this->kP);
        this->kI = (0-this->kI);
        this->kD = (0-this->kD);
    }
}

void PID::Compute() {
    
    //if PID is off (i.e. drone being controller manually), do not run function or compute values
    if (!inAuto) return;
    
    // Get the current time
    unsigned long long now = millis();

    // Compute the time change
    double timeChange = (double)(now - lastTime);

    if (timeChange >= sampleTime) {
        // Compute all the working error variables
        double error = setPoint - input;
        double dInput = (input - lastInput);
        outputSum += (kI * error);

        // Add Proportional on Measurement, if P_ON_M is specified
        if (!pOnE) outputSum -= kP * dInput;

        if(outputSum > outMax) outputSum = outMax;
        else if(outputSum < outMin) outputSum = outMin;

        // Add Proportional on Error, if P_ON_E is specified
        if (pOnE) output = kP * error;
        else output = 0;

        // Compute the output
        output += outputSum - kD * dInput;

        // Getting rid of windup lag
        if(output > outMax) output = outMax;
        else if(output < outMin) output = outMin;
        
        // Reset the needed variables
        lastInput = input;
        lastTime = now;
    }
}

int PID::GetSampleTime() {
    return sampleTime;
}

void PID::SetSampleTime(int newSampleTime){
   if (newSampleTime > 0) {
       double ratio = (double)newSampleTime / (double)sampleTime;
       kI *= ratio;
       kD /= ratio;
       sampleTime = newSampleTime;
   }
}

double PID::GetKP() {
    return kP;
}

void PID::SetKP(double kP) {
    this->kP = kP;
}

double PID::GetKI() {
    return kI;
}

void PID::SetKI(double kI) {
    this->kI = kI;
}

double PID::GetKD() {
    return kD;
}

void PID::SetKD(double kD) {
    this->kD = kD;
}

double PID::GetSetPoint() {
    return setPoint;
}

void PID::SetSetPoint(double setPoint) {
    this->setPoint = setPoint;
}

void PID::SetOutputLimits(double min, double max)
{
   if(min > max) return;
   outMin = min;
   outMax = max;
    
   if(output > outMax) output = outMax;
   else if(output < outMin) output = outMin;
 
   if(outputSum> outMax) outputSum = outMax;
   else if(outputSum < outMin) outputSum = outMin;
}

void PID::SetMode (bool mode){
    bool newAuto = (mode == AUTOMATIC);

    // If it goes from manual to auto
    if (newAuto == !inAuto){
        Initialize();
    }
    
    inAuto = newAuto;
}

void PID::Initialize (){
    // Keeps derivative from spiking when it switches from manual to automatic
    lastInput = input;

    outputSum = output; 
    if (outputSum > outMax)
        outputSum = outMax;
    else if (outputSum < outMin)
        outputSum = outMin;
}

void PID::SetControllerDirection(int direction) {
    controllerDirection = direction;
}

void PID::UpdateInput(double newInput) {
    input = newInput;
}

double PID::GetOutput() {
    return output;
}


