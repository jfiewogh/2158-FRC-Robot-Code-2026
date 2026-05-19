package org.austincans.lib.hardware.sensors.encoders;

public class AbsoluteEncoderConfig {
    public int deviceId;
    public double offsetRotations;
    public double discontinuityPoint;
    public boolean isInverted;
    public String canbus;
}
