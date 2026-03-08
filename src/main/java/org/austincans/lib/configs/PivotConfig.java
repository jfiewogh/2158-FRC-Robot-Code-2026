package org.austincans.lib.configs;

public record PivotConfig(
    MotorConfig motorConfig,
    AbsoluteEncoderConfig absoluteEncoderConfig,
    double gearRatio,
    double armLengthMeters,
    double minAngleRad,
    double maxAngleRad,
    double startingAngleRad,
    double maxVelocityRadPerSec,
    double maxAccelerationRadPerSecSq) {}