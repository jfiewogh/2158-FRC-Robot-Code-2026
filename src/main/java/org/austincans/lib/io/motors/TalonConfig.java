package org.austincans.lib.io.motors;

import org.austincans.lib.io.motors.TalonIO.TalonControlSlot;
import org.austincans.lib.util.math.physical.Gearbox;

import com.ctre.phoenix6.CANBus;

import edu.wpi.first.units.measure.Current;

public abstract class TalonConfig extends MotorConfig<TalonControlSlot> {
    public CANBus canBus;

    /* Current Limits */
    public Current statorCurrentLimit;
    public Current supplyCurrentLimit;

    /* Gearbox */
    public Gearbox rotorToRemoteSensor;

    /* Feedback Sensor */
    public int feedbackSensorSourceId; 
}
