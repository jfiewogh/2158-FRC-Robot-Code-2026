package org.austincans.lib.io.sensors.encoders.configs.requests;

import org.austincans.lib.io.ConfigRequests;
import org.austincans.lib.io.sensors.encoders.configs.CANcoderConfig;

import edu.wpi.first.units.measure.Angle;

public class CANcoderIOConfigRequests extends ConfigRequests<CANcoderConfig> {
    public CANcoderConfig config;
    public boolean magnetSensorConfigChanged;

    public void magnetOffset(Angle offset) {
        magnetSensorConfigChanged |= !config.magnetOffset.isEquivalent(offset);
        config.magnetOffset = offset;
    }
    public void absoluteSensorDiscontinuityPoint(Angle discontinuityPoint) {
        magnetSensorConfigChanged |= 
            !config.absoluteSensorDiscontinuityPoint.isEquivalent(discontinuityPoint);
        config.absoluteSensorDiscontinuityPoint = discontinuityPoint;
    }
    public void sensorDirectionInverted(boolean inverted) {
        magnetSensorConfigChanged |= config.sensorDirectionInverted != inverted;
        config.sensorDirectionInverted = inverted;
    }
}
