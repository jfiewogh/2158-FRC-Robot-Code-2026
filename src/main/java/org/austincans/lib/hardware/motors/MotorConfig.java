package org.austincans.lib.hardware.motors;

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

    public static class PIDConfig {
        public double kP; 
        public double kI;
        public double kD;
    }

    /** Feedforward Config */
    public static class FFConfig {
        public double kS;
        public double kG;
        public double kV;
        public double kA;
        public double kT;
    }

    /**
     * Values as recorded in REV client or Phoenix Tuner X. 
     * Soft limit values are in rotations and after gearing.
     */
    public static class SoftLimitConfig {
        public boolean forwardSoftLimitEnabled;
        public double forwardSoftLimitRotations;
        public boolean reverseSoftLimitEnabled;
        public double reverseSoftLimitRotations;
    }
}