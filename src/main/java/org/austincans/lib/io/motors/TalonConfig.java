package org.austincans.lib.io.motors;

import org.austincans.lib.io.motors.TalonIO.TalonControlSlot;

import com.ctre.phoenix6.CANBus;

import edu.wpi.first.units.measure.Current;

public class TalonConfig extends MotorConfig<TalonControlSlot> {
    public CANBus canBus;

    public Current statorCurrentLimit;
    public Current supplyCurrentLimit;

    public double sensorToMechanismRatio;
}
