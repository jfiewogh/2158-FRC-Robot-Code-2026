package org.austincans.lib.configs;

/** Add or modify variables if necessary */
public abstract class MotorConfig {
    public int deviceId;
    public boolean brakeModeEnabled;
    public boolean isInverted;
    public PIDConfig pidConfig;
    public FFConfig ffConfig;
    /** Values after gearing */
    public SoftLimitConfig softLimitConfig;
    /** Motor over mechanism. For example, 50 motor rotations over 2 mechanism rotation = 25. */
    public double gearRatio;
    /** Value after gearing */
    public double maxSpeedRotationsPerSec;
    /** Value after gearing */
    public double maxAccelerationRotationsPerSecSq;
}