package org.austincans.lib.configs;

public record AbsoluteEncoderConfig(
    int deviceId,
    double offsetRotations,
    double discontinuityPoint,
    boolean isInverted,
    String canbus // CTRE only
) {}
