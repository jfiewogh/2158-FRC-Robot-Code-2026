package org.austincans.lib.io.motors;

public class TalonConfig extends MotorConfig {
    public String canbus;
    public double statorCurrentLimitAmps;
    public double supplyCurrentLimitAmps;

    public double sensorToMechanismRatio;
}
