package org.austincans.lib.io.motors.inputs;

import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.units.measure.Current;
import edu.wpi.first.units.measure.Temperature;
import edu.wpi.first.units.measure.Voltage;

/**
 * Values universal for all motors
 * TODO: Current values are universal only for real motors. Eventually, update to be universal for real and sim.  
 */
public class MotorIOInputs {
    public Angle position;
    public AngularVelocity velocity;
    public Voltage voltage;
    public Current statorCurrent;
    public Temperature temperature;
}