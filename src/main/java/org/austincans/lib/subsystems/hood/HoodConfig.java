package org.austincans.lib.subsystems.hood;
import org.austincans.lib.encoders.AbsoluteEncoderConfig;
import org.austincans.lib.motors.MotorConfig;

public record HoodConfig(
    MotorConfig motorConfig,
    AbsoluteEncoderConfig encoderConfig,
    int ServoLinearId,
    double rotationalInertiaMetersSq,
    double gearing

) {}

