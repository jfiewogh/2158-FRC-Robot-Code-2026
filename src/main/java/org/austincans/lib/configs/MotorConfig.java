package org.austincans.lib.configs;

/** Add or modify variables if necessary */
public abstract class MotorConfig {
    public int deviceId;
    public boolean brakeModeEnabled;
    public boolean isInverted;
    public PIDConfig pidConfig;
    public FFConfig ffConfig;
    public SoftLimitConfig softLimitConfig; // before gearing
    public double gearRatio; // input over output // ex. 50 motor rotations / 1 mechanism rotation = 50
    public double maxSpeedRotationsPerSec; // after gearing
    public double maxAccelerationRotationsPerSecSq; // after gearing
}