package org.austincans.lib.motors;

/** Add or modify variables if necessary */
public record MotorConfig(
    int deviceId,
    boolean brakeModeEnabled,
    boolean isInverted,
    PIDFFConfig pidffConfig,
    SoftLimitConfig softLimitConfig,
    double statorCurrentLimit,
    boolean isBrushless, // only for Spark
    String canbus // only for Talon
) {}
