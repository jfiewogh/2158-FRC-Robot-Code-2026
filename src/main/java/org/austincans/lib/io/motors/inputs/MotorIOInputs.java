package org.austincans.lib.io.motors.inputs;

import org.austincans.lib.io.Inputs;

import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.units.measure.Current;
import edu.wpi.first.units.measure.Temperature;
import edu.wpi.first.units.measure.Voltage;

/* 
TODO: Current values are universal only for real motors. 
Eventually, update to be universal for real and sim. 
WPIlib sim motors do not have temperature
Bullet sim motors (hinges) likely work with torque and don't have voltage, current, or temperature
*/
/** Values universal for all motors */
public class MotorIOInputs implements Inputs {
    public Angle position;
    public AngularVelocity velocity;
    public Voltage voltage;
    public Current statorCurrent;
    public Temperature temperature;
}