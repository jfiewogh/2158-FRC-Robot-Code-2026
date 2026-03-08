package org.austincans.lib.subsystems.pivot;

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