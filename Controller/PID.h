#ifndef PID_H
#define PID_H

#if ARDUINO >= 100
    #include "Arduino.h"
#else
    #include "WProgram.h"
    #include "pins_arduino.h"
    #include "WConstants.h"
#endif

// author: Nishant Hooda
// date: 25-10-2017

const int MANUAL = 0;
const int AUTOMATIC = 1;

const int DIRECT = 0;
const int REVERSE = 1;

const int P_ON_M = 0;
const int P_ON_E = 1;

class PID {
private:
    /// The proportional, integral, and derivative constants respectively
    double kP, kI, kD;

    /// The last time recorded in milliseconds
    unsigned long long lastTime;

    /// The input from the sensors
    double input;

    /// The output that needs to be computed
    double output = 0;

    /// What we would like the current state of the object being controlled to be
    double setPoint;

    /// The total sum of error in the object state over time (used to calculate integral portion of output)
    double errSum = 0;

    /// The last error we recorded for the object state (used to calculate the derivative portion of output)
    double lastErr;

    // Tracks the last input for derivative on measurement
    double lastInput;

    /// The amount of time in milliseconds that should pass between successive calls to compute
    int sampleTime = 20;

    // Whether or not the PID is currently running 
    bool inAuto = true;

    // Tracking integral term solves tuning changes
    double iTerm;

    // Used for clamping the output
    double outMin, outMax;

    int controllerDirection = DIRECT;

    // Whether or not we are doing proportional on measurement
    bool pOnE = true;

    double outputSum = 0;
    
    
public:
    /**
    * Default constructor for PID
    */
    PID();

    /**
    * Constructor that takes the constants for the proportional, integral, and derivative portions
    * @param kP The proportional constant
    * @param kI The integral constant
    * @param kD The derivative constant
    */
    PID(double kP, double kI, double kD, double setPoint);

    /**
    * Setter for all of the constant
    * @param kP The new value of the proportional constant
    * @param kI The new value of the integral constant
    * @param kD The new value of the derivative constant
    * @param pOn How user wants the proportional to be calculated (1 for on error, 0 for on measurement)
    */
    void SetTunings(double kP, double kI, double kD, int pOn);

    /**
    * Computes the output of the PID given the current object state (characterized by gyro and accelerometer measurements)
    */
    void Compute();

    /**
    * Getter for the sample time.
    */
    int GetSampleTime();

    /**
    * Setters for the sample time. It also recomputes the constants as they are sample time dependent
    * @param newSampleTime the new sample time for the PID
    */
    void SetSampleTime(int newSampleTime);

    /**
    * Getter for the proportional constant
    */
    double GetKP();

    /**
    * Setter for the poportional constant
    * @param kP The new value of the proportional constant
    */
    void SetKP(double kP);

    /**
    * Getter for the integral constant
    */
    double GetKI();

    /**
    * Setter for the integral constant
    */
    void SetKI(double kI);

    /**
    * Getter for the derivative constant
    */
    double GetKD();

    /**
    * Setter for the derivative constant
    */
    void SetKD(double kD);

    /**
    * Getter for setPoint
    */
    double GetSetPoint();

    /**
    * Setter for setPoint
    */
    void SetSetPoint(double setPoint);

    /**
    * Clamps output
    */
    void SetOutputLimits(double min, double max);
    
    /**
    * Turning PID on and off using keywords MANUAL and AUTOMATIC
    */
    void SetMode(bool mode);

    /**
    * Function to keep the variables constant when going from manual back to automatic and then implementing PID on these newly initialized variables
    */
    void Initialize ();

    /**
    * Sets the direction of the controller
    */
    void SetControllerDirection(int direction);

    /**
    * Updates the current state of the object being controlled
    * @param newInput the current state of the object
    */
    void UpdateInput(double newInput);

    /**
    * Retrieves the output from the PID
    */
    double GetOutput();
};

#endif
