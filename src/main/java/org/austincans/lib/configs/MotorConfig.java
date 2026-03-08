package org.austincans.lib.configs;

/** Add or modify variables if necessary */
public record MotorConfig(
    int deviceId,
    boolean brakeModeEnabled,
    boolean isInverted,
    PIDConfig pidConfig,
    FFConfig ffConfig,
    SoftLimitConfig softLimitConfig, // before gearing
    double gearRatio, // input over output // ex. 50 motor rotations / 1 mechanism rotation = 50
    double maxSpeedRotationsPerSec, // after gearing
    double maxAccelerationRotationsPerSecSq, // after gearing
    double allowedClosedLoopErrorRad, // REV only // after gearing
    double statorCurrentLimitAmps,
    boolean isBrushless, // REV only
    String canbus // CTRE only
) {}