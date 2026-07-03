package org.austincans.lib.io.sensors.encoders.configs;

import org.austincans.lib.io.Config;

import edu.wpi.first.units.measure.Angle;

public class CANcoderConfig implements Config {
    public Angle magnetOffset;
    public Angle absoluteSensorDiscontinuityPoint;
    public boolean sensorDirectionInverted;
}
