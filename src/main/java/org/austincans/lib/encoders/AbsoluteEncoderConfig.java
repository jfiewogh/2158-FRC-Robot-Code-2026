package org.austincans.lib.encoders;

public record AbsoluteEncoderConfig(
    int deviceId,
    double offsetRotations,
    double discontinuityPoint,
    boolean isInverted,
    String canbus // CTRE only
) {}
