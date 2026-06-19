package org.austincans.lib.io.sensors.encoders;

import org.austincans.lib.io.DeviceIO;
import org.austincans.lib.io.ConfigurableIO;

import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.AngularVelocity;

public interface AbsoluteEncoderIO<
    InputsType extends AbsoluteEncoderIO.AbsoluteEncoderIOInputs,
    ConfigRequestsType extends AbsoluteEncoderIO.AbsoluteEncoderIOConfigRequests<ConfigType>,
    ConfigType>
extends 
    DeviceIO<InputsType>, 
    ConfigurableIO<ConfigRequestsType> 
{

    public static class AbsoluteEncoderIOInputs {
        public boolean connected;
        public Angle absolutePosition;
        public Angle position;
        public AngularVelocity velocity; 
    }

    public static class AbsoluteEncoderIOConfigRequests<ConfigType> {
        protected ConfigType config;
        protected boolean configChanged;

        public void config(ConfigType config) {
            configChanged = true;
            this.config = config;
        }
    }
}
