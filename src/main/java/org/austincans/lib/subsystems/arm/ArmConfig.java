package org.austincans.lib.subsystems.arm;

import org.austincans.lib.motors.PIDFFConfig;
import org.austincans.lib.motors.SoftLimitConfig;

public record ArmConfig(
    int motorId,
    String motorCanbus, // only for Talon
    int encoderId,
    String encoderCanbus, // only for Talon
    double gearRatio,
    double armLengthMeters,
    double minAngleRad,
    double maxAngleRad,
    double startingAngleRad,
    double maxVelocityRadPerSec,
    double maxAccelerationRadPerSecSq,
    double kP,
    double kI,
    double kD,
    double kS,
    double kG,
    double kV,
    double kA,
    double statorCurrentLimit,
    boolean brakeModeEnabled,
    boolean motorIsBrushless, // only for Spark
    boolean forwardSoftLimitEnabled,
    double forwardSoftLimit,
    boolean reverseSoftLimitEnabled,
    double reverseSoftLimit) 
{
    public PIDFFConfig pidffConfig() {
        return new PIDFFConfig(kP, kI, kD, kS, kG, kV, kA, 0);
    }
    public SoftLimitConfig softLimitConfig() {
        return new SoftLimitConfig(forwardSoftLimitEnabled, forwardSoftLimit, reverseSoftLimitEnabled, reverseSoftLimit);
    }
}