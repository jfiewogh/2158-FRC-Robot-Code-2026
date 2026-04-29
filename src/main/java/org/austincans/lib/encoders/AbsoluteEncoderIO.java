package org.austincans.lib.encoders;

import edu.wpi.first.units.measure.Angle;

public interface AbsoluteEncoderIO {
    public class AbsoluteEncoderIOInputs {
        public boolean connected = false;
        public double absolutePositionRad;
        public double positionRad;
        public double velocityRadPerSec;
    }

    public Angle getAbsolutePosition();

    public void updateInputs(AbsoluteEncoderIOInputs inputs);
}