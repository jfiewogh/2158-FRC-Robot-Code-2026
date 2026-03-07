package org.austincans.lib.motors;

import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.configs.TalonFXSConfiguration;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.config.SparkFlexConfig;
import com.revrobotics.spark.config.SparkMaxConfig;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;

/** This class should be used to create the MotorIO instances. */
public class MotorFactory {
    public static MotorIOSparkFlex createSparkFlex(int deviceId, boolean isBrushless, PIDFFConfig pidffConfig,
            SoftLimitConfig softLimitConfig, boolean brakeModeEnabled, SparkFlexConfig motorConfig) {
        return new MotorIOSparkFlex(
                deviceId, 
                isBrushless ? MotorType.kBrushless : MotorType.kBrushed, 
                pidffConfig, 
                softLimitConfig,
                brakeModeEnabled ? IdleMode.kBrake : IdleMode.kCoast,
                motorConfig);
    }

    public static MotorIOSparkMax createSparkMax(int deviceId, boolean isBrushless, PIDFFConfig pidffConfig, 
            SoftLimitConfig softLimitConfig, boolean brakeModeEnabled, SparkMaxConfig motorConfig) {
        return new MotorIOSparkMax(
                deviceId, 
                isBrushless ? MotorType.kBrushless : MotorType.kBrushed, 
                pidffConfig, 
                softLimitConfig,
                brakeModeEnabled ? IdleMode.kBrake : IdleMode.kCoast,
                motorConfig);
    }

    public static MotorIOTalonFX createTalonFX(int deviceId, String canbus, TalonFXConfiguration motorConfig, PIDFFConfig pidffConfig,
            SoftLimitConfig softLimitConfig, NeutralModeValue nuetralMode, InvertedValue isInverted) {
        return new MotorIOTalonFX(deviceId, canbus, motorConfig, pidffConfig,softLimitConfig,nuetralMode,isInverted);
    }

    public static MotorIOTalonFXS createTalonFXS(int deviceId, String canbus, TalonFXSConfiguration config, PIDFFConfig pidffConfig,
            SoftLimitConfig softLimitConfig, NeutralModeValue nuetralMode, InvertedValue isInverted) {
        return new MotorIOTalonFXS(deviceId, canbus, config, pidffConfig,softLimitConfig,nuetralMode,isInverted);
    }
}
