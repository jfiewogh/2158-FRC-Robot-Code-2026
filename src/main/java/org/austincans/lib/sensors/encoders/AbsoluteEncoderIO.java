package org.austincans.lib.sensors.encoders;

import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.AngularVelocity;

public interface AbsoluteEncoderIO {
    public class AbsoluteEncoderIOInputs {
        public boolean connected = false;
        public Angle absolutePosition;
        public Angle position;
        public AngularVelocity velocity;
    }

    public void updateInputs(AbsoluteEncoderIOInputs inputs);
}