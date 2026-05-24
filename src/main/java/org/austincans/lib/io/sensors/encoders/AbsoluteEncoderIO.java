package org.austincans.lib.io.sensors.encoders;

import org.austincans.lib.io.DeviceIO;

import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.AngularVelocity;

public interface AbsoluteEncoderIO extends DeviceIO<AbsoluteEncoderIO.AbsoluteEncoderIOInputs> {
    public class AbsoluteEncoderIOInputs extends DeviceIO.DeviceIOInputs {
        public boolean connected = false;
        public Angle absolutePosition;
        public Angle position;
        public AngularVelocity velocity;
    }
}