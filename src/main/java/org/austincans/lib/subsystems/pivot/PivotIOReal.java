package org.austincans.lib.subsystems.pivot;

import org.austincans.lib.motors.MotorIOReal;
import org.austincans.lib.sensors.encoders.AbsoluteEncoderIO;

public abstract class PivotIOReal<T extends MotorIOReal, U extends AbsoluteEncoderIO> implements PivotIO {
    protected final T motorIO;
    protected final U absoluteEncoderIO;

    protected PivotIOReal(T motorIO, U absoluteEncoderIO) {
        this.motorIO = motorIO;
        this.absoluteEncoderIO = absoluteEncoderIO;
    }

    @Override
    public void updateInputs(PivotIOInputs inputs) {
        motorIO.updateInputs(inputs.motorInputs);
        absoluteEncoderIO.updateInputs(inputs.absoluteEncoderInputs);
    }
    
    public void applyOutputs(PivotIOOutputs outputs){
        motorIO.applyOutputs(outputs.motorOutputs);
    }
}
