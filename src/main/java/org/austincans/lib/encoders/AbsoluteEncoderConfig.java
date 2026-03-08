package org.austincans.lib.encoders;

public record AbsoluteEncoderConfig(
    int deviceId,
    double offsetRad,
    boolean isInverted,
    String canbus // CTRE only
) {}
