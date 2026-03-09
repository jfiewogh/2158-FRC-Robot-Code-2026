package org.austincans.lib.encoders;

public abstract interface AbsoluteEncoderIO {
    public class AbsoluteEncoderIOInputs {
        public boolean connected = false;
        public double absolutePositionRad;
        public double positionRad;
        public double velocityRadPerSec;
    }

    public void updateInputs(AbsoluteEncoderIOInputs inputs);

    public double getAbsolutePositionRotations();
}
