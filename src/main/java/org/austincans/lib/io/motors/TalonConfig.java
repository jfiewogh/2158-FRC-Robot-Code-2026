package org.austincans.lib.io.motors;

import org.austincans.lib.util.math.physical.Gearbox;

import edu.wpi.first.units.measure.AngularAcceleration;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.units.measure.Current;

public abstract class TalonConfig extends MotorConfig {
    public ControlConfig slot0Config = new ControlConfig();
    public ControlConfig slot2Config = new ControlConfig();
    public ControlConfig slot1Config = new ControlConfig();

    public AngularVelocity motionMagicMechanismCruiseVelocity;
    public AngularAcceleration motionMagicMechanismMaxAcceleration;

    /* Current Limits */
    public Current statorCurrentLimit;
    public Current supplyCurrentLimit;

    /* Gearbox */
    public Gearbox rotorToRemoteSensor;

    /* Feedback Sensor */
    public int feedbackSensorSourceId; 
}
