package org.austincans.lib.subsystems.pivot;

import org.austincans.lib.encoders.AbsoluteEncoderIO;
import org.austincans.lib.motors.MotorIOReal;

public abstract class PivotIOReal<T extends MotorIOReal, U extends AbsoluteEncoderIO> implements PivotIO {
    protected final T motorIO;
    protected final U absoluteEncoderIO;

    protected PivotIOReal(T motorIO, U absoluteEncoderIO) {
        this.motorIO = motorIO;
        this.absoluteEncoderIO = absoluteEncoderIO;
    }

    @Override
    public void updateInputs(ArmIOInputs inputs) {
        motorIO.updateInputs(inputs.motorInputs);
        absoluteEncoderIO.updateInputs(inputs.absoluteEncoderInputs);
    }    

    @Override
    public void setVoltage(double voltage) {
        motorIO.setVoltage(voltage);
    }

    @Override
    public void setPosition(double positionRad) {
        motorIO.setPosition(positionRad);
    }

    @Override
    public void setVelocity(double velocityRadPerSec) {
        motorIO.setVelocity(velocityRadPerSec);
    }
}
