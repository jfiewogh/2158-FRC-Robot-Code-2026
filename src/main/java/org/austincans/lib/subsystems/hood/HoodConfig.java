package org.austincans.lib.subsystems.hood;
import org.austincans.lib.configs.AbsoluteEncoderConfig;
import org.austincans.lib.configs.MotorConfig;

public record HoodConfig(
    MotorConfig motorConfig,
    AbsoluteEncoderConfig encoderConfig,
    int ServoLinearId,
    double rotationalInertiaMetersSq,
    double gearing

) {}

