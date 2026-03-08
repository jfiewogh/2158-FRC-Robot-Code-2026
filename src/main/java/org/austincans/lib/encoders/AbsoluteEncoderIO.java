package org.austincans.lib.encoders;

import org.littletonrobotics.junction.AutoLog;

public abstract interface AbsoluteEncoderIO {
    @AutoLog
    public class AbsoluteEncoderIOInputs {
        public boolean connected = false;
        public double absolutePositionRad;
        public double positionRad;
        public double velocityRadPerSec;
    }

    public void updateInputs(AbsoluteEncoderIOInputs inputs);
}
