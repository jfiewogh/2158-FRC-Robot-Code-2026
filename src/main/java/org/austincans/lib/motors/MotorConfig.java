package org.austincans.lib.motors;

public record MotorConfig(
    int deviceId,
    boolean brakeModeEnabled,
    boolean isBrushless, // only for Spark
    PIDFFConfig pidffConfig,
    SoftLimitConfig softLimitConfig,
    double statorCurrentLimit
) {}
