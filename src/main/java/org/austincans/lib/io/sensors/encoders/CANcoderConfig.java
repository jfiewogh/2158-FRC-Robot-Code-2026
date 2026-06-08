package org.austincans.lib.io.sensors.encoders;

import edu.wpi.first.units.measure.Angle;

public class CANcoderConfig {
    public Angle magnetOffset;
    public Angle absoluteSensorDiscontinuityPoint;
    public boolean sensorDirectionInverted;
}
