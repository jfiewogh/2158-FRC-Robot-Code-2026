package org.austincans.lib.hardware.sensors.encoders;

import org.austincans.lib.hardware.HardwareIO;

import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.AngularVelocity;

public interface AbsoluteEncoderIO extends HardwareIO<AbsoluteEncoderIO.AbsoluteEncoderIOInputs> {
    public class AbsoluteEncoderIOInputs {
        public boolean connected = false;
        public Angle absolutePosition;
        public Angle position;
        public AngularVelocity velocity;
    }
}