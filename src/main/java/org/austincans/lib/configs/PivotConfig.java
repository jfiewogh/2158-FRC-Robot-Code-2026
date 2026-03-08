package org.austincans.lib.configs;

import org.austincans.lib.encoders.AbsoluteEncoderConfig;
import org.austincans.lib.motors.MotorConfig;

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