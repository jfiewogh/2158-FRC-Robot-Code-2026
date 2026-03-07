package org.austincans.lib.motors;

import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.configs.TalonFXSConfiguration;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.config.SparkFlexConfig;
import com.revrobotics.spark.config.SparkMaxConfig;
import com.revrobotics.spark.config.SparkBaseConfig;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;

/** This class should be used to create the MotorIO instances. */
public class MotorFactory {
    private static <T extends SparkBaseConfig> T applyMotorConfigToSpark(T config, MotorConfig motorConfig) {
      PIDFFConfig pidffConfig = motorConfig.pidffConfig();
      config.closedLoop.p(pidffConfig.kP());
      config.closedLoop.i(pidffConfig.kI());
      config.closedLoop.d(pidffConfig.kD());
      config.closedLoop.feedForward.kS(pidffConfig.kS());
      config.closedLoop.feedForward.kG(pidffConfig.kG());
      config.closedLoop.feedForward.kV(pidffConfig.kV());
      config.closedLoop.feedForward.kA(pidffConfig.kA());
      SoftLimitConfig softLimitConfig = motorConfig.softLimitConfig();
      config.softLimit.forwardSoftLimitEnabled(softLimitConfig.forwardSoftLimitEnabled());
      config.softLimit.forwardSoftLimit(softLimitConfig.forwardSoftLimit());
      config.softLimit.reverseSoftLimitEnabled(softLimitConfig.reverseSoftLimitEnabled());
      config.softLimit.reverseSoftLimit(softLimitConfig.reverseSoftLimit());
      config.idleMode(motorConfig.brakeModeEnabled() ? IdleMode.kBrake : IdleMode.kCoast);
      config.smartCurrentLimit((int) motorConfig.statorCurrentLimit());
      return config;
    }

    public static MotorIOSparkFlex createSparkFlex(MotorConfig motorConfig) {
      return new MotorIOSparkFlex(
          motorConfig.deviceId(),
          motorConfig.isBrushless() ? MotorType.kBrushless : MotorType.kBrushed,
          applyMotorConfigToSpark(new SparkFlexConfig(), motorConfig));
    }

    public static MotorIOSparkMax createSparkMax(MotorConfig motorConfig) {
      return new MotorIOSparkMax(
          motorConfig.deviceId(),
          motorConfig.isBrushless() ? MotorType.kBrushless : MotorType.kBrushed,
          applyMotorConfigToSpark(new SparkMaxConfig(), motorConfig));
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
