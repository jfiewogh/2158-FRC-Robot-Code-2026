package org.austincans.lib.configs;

public record ArmConfig(
    MotorConfig motorConfig,
    AbsoluteEncoderConfig absoluteEncoderConfig,
    double gearRatio,
    double armLengthMeters,
    double minAngleRad,
    double maxAngleRad,
    double startingAngleRad,
    double maxVelocityRadPerSec,
    double maxAccelerationRadPerSecSq) {}