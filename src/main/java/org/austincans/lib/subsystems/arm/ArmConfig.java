package org.austincans.lib.subsystems.arm;

public record ArmConfig(
    int motorId,
    String motorCanbus, // only for Talon
    int encoderId,
    String encoderCanbus,  // only for Talon
    double gearRatio,
    double armLength,
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
    double statorCurrentLimit, // only for Talon
    boolean brakeModeEnabled,
    boolean forwardSoftLimitEnabled,
    double forwardSoftLimit,
    boolean reverseSoftLimitEnabled,
    double reverseSoftLimit
) {}