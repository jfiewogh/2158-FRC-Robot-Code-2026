package org.austincans.lib.encoders;

import org.littletonrobotics.junction.AutoLog;

public abstract interface EncoderIO {
    @AutoLog
    public class EncoderIOInputs {
        double positionRad;
        double velocityRadPerSec;
    }   

    public void updateInputs(EncoderIOInputs inputs);

    public double getPositionRad();

    public double getVelocityRadPerSec();
}
