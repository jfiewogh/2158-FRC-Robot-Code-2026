package org.austincans.lib.io.sensors.encoders.configs.requests;

import org.austincans.lib.io.ConfigRequests;
import org.austincans.lib.io.sensors.encoders.configs.CANcoderConfig;

import edu.wpi.first.units.measure.Angle;

public class CANcoderIOConfigRequests extends ConfigRequests {
    public boolean magnetSensorConfigChanged;

    public void magnetOffset(Angle offset) {
        CANcoderConfig canCoderConfig = (CANcoderConfig) config;
        magnetSensorConfigChanged |= !canCoderConfig.magnetOffset.isEquivalent(offset);
        canCoderConfig.magnetOffset = offset;
    }
    public void absoluteSensorDiscontinuityPoint(Angle discontinuityPoint) {
        CANcoderConfig canCoderConfig = (CANcoderConfig) config;
        magnetSensorConfigChanged |= 
            !canCoderConfig.absoluteSensorDiscontinuityPoint.isEquivalent(discontinuityPoint);
        canCoderConfig.absoluteSensorDiscontinuityPoint = discontinuityPoint;
    }
    public void sensorDirectionInverted(boolean inverted) {
        CANcoderConfig canCoderConfig = (CANcoderConfig) config;
        magnetSensorConfigChanged |= canCoderConfig.sensorDirectionInverted != inverted;
        canCoderConfig.sensorDirectionInverted = inverted;
    }
}
