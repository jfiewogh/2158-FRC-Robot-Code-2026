package org.austincans.lib.motors;

import org.austincans.lib.configs.FFConfig;
import org.austincans.lib.configs.PIDConfig;
import org.austincans.lib.configs.SoftLimitConfig;
import com.ctre.phoenix6.CANBus;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.configs.TalonFXSConfiguration;
import com.ctre.phoenix6.signals.ExternalFeedbackSensorSourceValue;
import com.ctre.phoenix6.signals.FeedbackSensorSourceValue;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;
import com.revrobotics.spark.ClosedLoopSlot;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.config.SparkBaseConfig;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;
import com.revrobotics.spark.config.SparkFlexConfig;
import com.revrobotics.spark.config.SparkMaxConfig;

/** This class should be used to create the MotorIO instances. */
public class MotorFactory {
  /* REV */

  private static <T extends SparkBaseConfig> T applyMotorConfigToSparkConfig(T config, MotorConfig motorConfig) {
    config.encoder.positionConversionFactor(motorConfig.gearRatio());
    PIDConfig pidConfig = motorConfig.pidConfig();
    config.closedLoop.p(pidConfig.kP());
    config.closedLoop.i(pidConfig.kI());
    config.closedLoop.d(pidConfig.kD());
    FFConfig ffConfig = motorConfig.ffConfig();
    config.closedLoop.feedForward.kS(ffConfig.kS());
    config.closedLoop.feedForward.kG(ffConfig.kG());
    config.closedLoop.feedForward.kV(ffConfig.kV());
    config.closedLoop.feedForward.kA(ffConfig.kA());
    config.closedLoop.maxMotion.cruiseVelocity(motorConfig.maxSpeedRotationsPerSec() * 60);
    config.closedLoop.maxMotion.maxAcceleration(motorConfig.maxAccelerationRotationsPerSecSq());
    config.closedLoop.allowedClosedLoopError(motorConfig.allowedClosedLoopErrorRad(), ClosedLoopSlot.kSlot0);
    SoftLimitConfig softLimitConfig = motorConfig.softLimitConfig();
    config.softLimit.forwardSoftLimitEnabled(softLimitConfig.forwardSoftLimitEnabled());
    config.softLimit.forwardSoftLimit(softLimitConfig.forwardSoftLimit());
    config.softLimit.reverseSoftLimitEnabled(softLimitConfig.reverseSoftLimitEnabled());
    config.softLimit.reverseSoftLimit(softLimitConfig.reverseSoftLimit());
    config.idleMode(motorConfig.brakeModeEnabled() ? IdleMode.kBrake : IdleMode.kCoast);
    config.inverted(motorConfig.isInverted());
    config.smartCurrentLimit((int) motorConfig.statorCurrentLimitAmps());
    return config;
  }

  private static MotorIOSparkFlex createSparkFlex(MotorConfig motorConfig, SparkFlexConfig sparkFlexConfig) {
    return new MotorIOSparkFlex(
        motorConfig.deviceId(),
        motorConfig.isBrushless() ? MotorType.kBrushless : MotorType.kBrushed,
        applyMotorConfigToSparkConfig(sparkFlexConfig, motorConfig));
  }

  public static MotorIOSparkFlex createSparkFlex(MotorConfig motorConfig) {
    return createSparkFlex(motorConfig, new SparkFlexConfig());
  }

  public static MotorIOSparkFlex createSparkFlexFollower(MotorConfig motorConfig, int leaderCanId) {
    SparkFlexConfig config = new SparkFlexConfig();
    config.follow(leaderCanId);
    return createSparkFlex(motorConfig, config);
  }

  private static MotorIOSparkMax createSparkMax(MotorConfig motorConfig, SparkMaxConfig sparkMaxConfig) {
    return new MotorIOSparkMax(
        motorConfig.deviceId(),
        motorConfig.isBrushless() ? MotorType.kBrushless : MotorType.kBrushed,
        applyMotorConfigToSparkConfig(sparkMaxConfig, motorConfig));
  }

  public static MotorIOSparkMax createSparkMax(MotorConfig motorConfig) {
    return createSparkMax(motorConfig, new SparkMaxConfig());
  }

  public static MotorIOSparkMax createSparkMaxFollower(MotorConfig motorConfig, int leaderCanId) {
    SparkMaxConfig config = new SparkMaxConfig();
    config.follow(leaderCanId);
    return createSparkMax(motorConfig, config);
  }

  /* CTRE */

  private static MotorIOTalonFX createTalonFX(MotorConfig motorConfig, TalonFXConfiguration config) {
    config.MotionMagic.MotionMagicCruiseVelocity = motorConfig.maxSpeedRotationsPerSec();
    config.MotionMagic.MotionMagicAcceleration = motorConfig.maxAccelerationRotationsPerSecSq();
    PIDConfig pidConfig = motorConfig.pidConfig();
    config.Slot0.kP = pidConfig.kP();
    config.Slot0.kI = pidConfig.kI();
    config.Slot0.kD = pidConfig.kD();
    FFConfig ffConfig = motorConfig.ffConfig();
    config.Slot0.kS = ffConfig.kS();
    config.Slot0.kG = ffConfig.kG();
    config.Slot0.kV = ffConfig.kV();
    config.Slot0.kA = ffConfig.kA();
    SoftLimitConfig softLimitConfig = motorConfig.softLimitConfig();
    config.SoftwareLimitSwitch.ForwardSoftLimitEnable = softLimitConfig.forwardSoftLimitEnabled();
    config.SoftwareLimitSwitch.ForwardSoftLimitThreshold = softLimitConfig.forwardSoftLimit();
    config.SoftwareLimitSwitch.ReverseSoftLimitEnable = softLimitConfig.reverseSoftLimitEnabled();
    config.SoftwareLimitSwitch.ReverseSoftLimitThreshold = softLimitConfig.reverseSoftLimit();
    config.MotorOutput.NeutralMode = motorConfig.brakeModeEnabled() ? NeutralModeValue.Brake : NeutralModeValue.Coast;
    config.MotorOutput.Inverted = motorConfig.isInverted() ? InvertedValue.Clockwise_Positive
        : InvertedValue.CounterClockwise_Positive;
    config.CurrentLimits.StatorCurrentLimit = motorConfig.statorCurrentLimitAmps();
    return new MotorIOTalonFX(motorConfig.deviceId(), new CANBus(motorConfig.canbus()), config);
  }

  /** Create a TalonFX motor using its internal sensor. */
  public static MotorIOTalonFX createTalonFX(MotorConfig motorConfig) {
    TalonFXConfiguration config = new TalonFXConfiguration();
    config.Feedback.FeedbackSensorSource = FeedbackSensorSourceValue.RotorSensor;
    config.Feedback.SensorToMechanismRatio = motorConfig.gearRatio();
    return createTalonFX(motorConfig, config);
  }

  /** Motor and CANcoder must be on same bus. */
  public static MotorIOTalonFX createTalonFXWithRemoteCANcoder(MotorConfig motorConfig, int cancoderId) {
    TalonFXConfiguration config = new TalonFXConfiguration();
    config.Feedback.FeedbackRemoteSensorID = cancoderId;
    config.Feedback.FeedbackSensorSource = FeedbackSensorSourceValue.RemoteCANcoder;
    return createTalonFX(motorConfig, config);
  }

  private static MotorIOTalonFXS createTalonFXS(MotorConfig motorConfig, TalonFXSConfiguration config) {
    config.MotionMagic.MotionMagicCruiseVelocity = motorConfig.maxSpeedRotationsPerSec();
    config.MotionMagic.MotionMagicAcceleration = motorConfig.maxAccelerationRotationsPerSecSq();
    PIDConfig pidConfig = motorConfig.pidConfig();
    config.Slot0.kP = pidConfig.kP();
    config.Slot0.kI = pidConfig.kI();
    config.Slot0.kD = pidConfig.kD();
    FFConfig ffConfig = motorConfig.ffConfig();
    config.Slot0.kS = ffConfig.kS();
    config.Slot0.kG = ffConfig.kG();
    config.Slot0.kV = ffConfig.kV();
    config.Slot0.kA = ffConfig.kA();
    SoftLimitConfig softLimitConfig = motorConfig.softLimitConfig();
    config.SoftwareLimitSwitch.ForwardSoftLimitEnable = softLimitConfig.forwardSoftLimitEnabled();
    config.SoftwareLimitSwitch.ForwardSoftLimitThreshold = softLimitConfig.forwardSoftLimit();
    config.SoftwareLimitSwitch.ReverseSoftLimitEnable = softLimitConfig.reverseSoftLimitEnabled();
    config.SoftwareLimitSwitch.ReverseSoftLimitThreshold = softLimitConfig.reverseSoftLimit();
    config.MotorOutput.NeutralMode = motorConfig.brakeModeEnabled() ? NeutralModeValue.Brake : NeutralModeValue.Coast;
    config.MotorOutput.Inverted = motorConfig.isInverted() ? InvertedValue.Clockwise_Positive
        : InvertedValue.CounterClockwise_Positive;
    config.CurrentLimits.StatorCurrentLimit = motorConfig.statorCurrentLimitAmps();
    return new MotorIOTalonFXS(motorConfig.deviceId(), new CANBus(motorConfig.canbus()), config);
  }

  public static MotorIOTalonFXS createTalonFXS(MotorConfig motorConfig, int cancoderId) {
    TalonFXSConfiguration config = new TalonFXSConfiguration();
    config.ExternalFeedback.ExternalFeedbackSensorSource = ExternalFeedbackSensorSourceValue.Commutation;
    config.ExternalFeedback.SensorToMechanismRatio = motorConfig.gearRatio();
    return createTalonFXS(motorConfig, config);
  }

  public static MotorIOTalonFXS createTalonFXSWithRemoteCANcoder(MotorConfig motorConfig, int cancoderId) {
    TalonFXSConfiguration config = new TalonFXSConfiguration();
    config.ExternalFeedback.ExternalFeedbackSensorSource = ExternalFeedbackSensorSourceValue.RemoteCANcoder;
    config.ExternalFeedback.FeedbackRemoteSensorID = cancoderId;
    return createTalonFXS(motorConfig, config);
  }
}
