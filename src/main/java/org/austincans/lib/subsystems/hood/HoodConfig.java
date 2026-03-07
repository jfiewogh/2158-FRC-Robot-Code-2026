package org.austincans.lib.subsystems.hood;

import org.austincans.lib.motors.PIDFFConfig;
import org.austincans.lib.motors.SoftLimitConfig;

import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;

public record HoodConfig(
    int deviceID,
    int ServoLinearId,
    double rotationalInertiaMetersSq,
    double gearing,
    double kP,
    double kI,
    double kD,
    double kS,
    double kG,
    double kV,
    double kA,
    boolean brakeModeEnabled,
    boolean motorIsBrushless, // only for Spark
    boolean forwardSoftLimitEnabled,
    double forwardSoftLimit,
    boolean reverseSoftLimitEnabled,
    double reverseSoftLimit,
    String motorCanbus, // only for Talon
    String encoderCanbus, // only for Talon
    NeutralModeValue nuetralMode,
    InvertedValue isInverted
)
{
    public PIDFFConfig pidffConfig() {
        return new PIDFFConfig(kP, kI, kD, kS, kG, kV, kA, 0);
    }
    public SoftLimitConfig softLimitConfig() {
        return new SoftLimitConfig(forwardSoftLimitEnabled, forwardSoftLimit, reverseSoftLimitEnabled, reverseSoftLimit);
    }
}


