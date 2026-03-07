package org.austincans.lib.motors;

import com.ctre.phoenix6.CANBus;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.configs.TalonFXSConfiguration;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;
import com.revrobotics.spark.SparkFlex;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.config.SparkBaseConfig;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;
import com.revrobotics.spark.config.SparkFlexConfig;
import com.revrobotics.spark.config.SparkMaxConfig;

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
      config.inverted(motorConfig.isInverted());
      config.smartCurrentLimit((int) motorConfig.statorCurrentLimit());
      return config;
    }

    public static MotorIOSparkFlex createSparkFlex(MotorConfig motorConfig, SparkFlexConfig config) {
      return new MotorIOSparkFlex(
          motorConfig.deviceId(),
          motorConfig.isBrushless() ? MotorType.kBrushless : MotorType.kBrushed,
          applyMotorConfigToSpark(config, motorConfig));
    }

    public static MotorIOSparkFlex createSparkFlex(MotorConfig motorConfig) {
      return createSparkFlex(motorConfig, new SparkFlexConfig());
    }

    public static MotorIOSparkFlex createSparkFlexFollower(MotorConfig motorConfig, SparkFlex leader) {
      SparkFlexConfig config = new SparkFlexConfig();
      config.follow(leader);
      return createSparkFlex(motorConfig, config);
    }

    public static MotorIOSparkMax createSparkMax(MotorConfig motorConfig) {
      return new MotorIOSparkMax(
          motorConfig.deviceId(),
          motorConfig.isBrushless() ? MotorType.kBrushless : MotorType.kBrushed,
          applyMotorConfigToSpark(new SparkMaxConfig(), motorConfig));
    }

    public static MotorIOTalonFX createTalonFX(MotorConfig motorConfig) {
      TalonFXConfiguration config = new TalonFXConfiguration();
      PIDFFConfig pidffConfig = motorConfig.pidffConfig();
      config.Slot0.kP = pidffConfig.kP();
      config.Slot0.kI = pidffConfig.kI();
      config.Slot0.kD = pidffConfig.kD();
      config.Slot0.kS = pidffConfig.kS();
      config.Slot0.kG = pidffConfig.kG();
      config.Slot0.kV = pidffConfig.kV();
      config.Slot0.kA = pidffConfig.kA();
      SoftLimitConfig softLimitConfig = motorConfig.softLimitConfig();
      config.SoftwareLimitSwitch.ForwardSoftLimitEnable = softLimitConfig.forwardSoftLimitEnabled();
      config.SoftwareLimitSwitch.ForwardSoftLimitThreshold = softLimitConfig.forwardSoftLimit();
      config.SoftwareLimitSwitch.ReverseSoftLimitEnable = softLimitConfig.reverseSoftLimitEnabled();
      config.SoftwareLimitSwitch.ReverseSoftLimitThreshold = softLimitConfig.reverseSoftLimit();
      config.MotorOutput.NeutralMode = motorConfig.brakeModeEnabled() ? NeutralModeValue.Brake : NeutralModeValue.Coast;
      config.MotorOutput.Inverted = motorConfig.isInverted() ? InvertedValue.Clockwise_Positive : InvertedValue.CounterClockwise_Positive;
      config.CurrentLimits.StatorCurrentLimit = motorConfig.statorCurrentLimit();
      return new MotorIOTalonFX(motorConfig.deviceId(), new CANBus(motorConfig.canbus()), config);
    }

    public static MotorIOTalonFXS createTalonFXS(MotorConfig motorConfig) {
      TalonFXSConfiguration config = new TalonFXSConfiguration();
      PIDFFConfig pidffConfig = motorConfig.pidffConfig();
      config.Slot0.kP = pidffConfig.kP();
      config.Slot0.kI = pidffConfig.kI();
      config.Slot0.kD = pidffConfig.kD();
      config.Slot0.kS = pidffConfig.kS();
      config.Slot0.kG = pidffConfig.kG();
      config.Slot0.kV = pidffConfig.kV();
      config.Slot0.kA = pidffConfig.kA();
      SoftLimitConfig softLimitConfig = motorConfig.softLimitConfig();
      config.SoftwareLimitSwitch.ForwardSoftLimitEnable = softLimitConfig.forwardSoftLimitEnabled();
      config.SoftwareLimitSwitch.ForwardSoftLimitThreshold = softLimitConfig.forwardSoftLimit();
      config.SoftwareLimitSwitch.ReverseSoftLimitEnable = softLimitConfig.reverseSoftLimitEnabled();
      config.SoftwareLimitSwitch.ReverseSoftLimitThreshold = softLimitConfig.reverseSoftLimit();
      config.MotorOutput.NeutralMode = motorConfig.brakeModeEnabled() ? NeutralModeValue.Brake : NeutralModeValue.Coast;
      config.MotorOutput.Inverted = motorConfig.isInverted() ? InvertedValue.Clockwise_Positive : InvertedValue.CounterClockwise_Positive;
      config.CurrentLimits.StatorCurrentLimit = motorConfig.statorCurrentLimit();
      return new MotorIOTalonFXS(motorConfig.deviceId(), new CANBus(motorConfig.canbus()), config);
    }
}
