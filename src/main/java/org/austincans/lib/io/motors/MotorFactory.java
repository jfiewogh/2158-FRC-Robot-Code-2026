package org.austincans.lib.io.motors;

import static edu.wpi.first.units.Units.Rotations;

import org.austincans.lib.io.motors.MotorConfig.*;

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
import com.revrobotics.spark.config.SparkFlexConfig;
import com.revrobotics.spark.config.SparkMaxConfig;

/** 
 * This class should be used to create the MotorIO instances. 
 * TODO: This class should be updated so that gear ratios are not set in the motor but are limited
 */
public class MotorFactory {
  /* REV */

  private static <ConfigType extends SparkBaseConfig> ConfigType applyMotorConfigToSparkConfig(ConfigType config, SparkConfig motorConfig) {
    /* Conversion factor from gearbox should be handled in the IO (convert motor values to mechanism values instead) */
    // double conversionFactor = motorConfig.gearbox;
    // config.encoder.positionConversionFactor(conversionFactor);
    // config.encoder.velocityConversionFactor(conversionFactor);
    
    PIDConfig pidConfig = motorConfig.pidConfig;
    config.closedLoop.p(pidConfig.kP);
    config.closedLoop.i(pidConfig.kI);
    config.closedLoop.d(pidConfig.kD);

    FFConfig ffConfig = motorConfig.ffConfig;
    config.closedLoop.feedForward.kS(ffConfig.kS);
    config.closedLoop.feedForward.kG(ffConfig.kG);
    config.closedLoop.feedForward.kV(ffConfig.kV);
    config.closedLoop.feedForward.kA(ffConfig.kA);
    // config.closedLoop.maxMotion.cruiseVelocity(motorConfig.maxSpeedRotationsPerSec * 60);
    // config.closedLoop.maxMotion.maxAcceleration(motorConfig.maxAccelerationRotationsPerSecSq);
    config.closedLoop.allowedClosedLoopError(motorConfig.allowedClosedLoopErrorRad, ClosedLoopSlot.kSlot0);
    
    SoftLimitConfig softLimitConfig = motorConfig.softLimitConfig;
    config.softLimit.forwardSoftLimitEnabled(softLimitConfig.forwardSoftLimitEnabled);
    config.softLimit.forwardSoftLimit(softLimitConfig.forwardSoftLimit.in(Rotations));
    config.softLimit.reverseSoftLimitEnabled(softLimitConfig.reverseSoftLimitEnabled);
    config.softLimit.reverseSoftLimit(softLimitConfig.reverseSoftLimit.in(Rotations));
    // config.idleMode(motorConfig.brakeModeEnabled ? IdleMode.kBrake : IdleMode.kCoast);
    config.inverted(motorConfig.isInverted);
    config.smartCurrentLimit(motorConfig.smartCurrentStallLimitAmps);
    
    return config;
  }

  public static SparkFlexIO createSparkFlex(SparkConfig motorConfig, SparkFlexConfig sparkFlexConfig) {
    return new SparkFlexIO(
        motorConfig.deviceId,
        motorConfig.isBrushless ? MotorType.kBrushless : MotorType.kBrushed,
        applyMotorConfigToSparkConfig(sparkFlexConfig, motorConfig));
  }

  public static SparkFlexIO createSparkFlex(SparkConfig motorConfig) {
    return createSparkFlex(motorConfig, new SparkFlexConfig());
  }

  public static SparkFlexIO createSparkFlexFollower(SparkConfig motorConfig, int leaderCanId) {
    SparkFlexConfig config = new SparkFlexConfig();
    config.follow(leaderCanId);
    return createSparkFlex(motorConfig, config);
  }

  private static SparkMaxIO createSparkMax(SparkConfig motorConfig, SparkMaxConfig sparkMaxConfig) {
    return new SparkMaxIO(
        motorConfig.deviceId,
        motorConfig.isBrushless ? MotorType.kBrushless : MotorType.kBrushed,
        applyMotorConfigToSparkConfig(sparkMaxConfig, motorConfig));
  }

  public static SparkMaxIO createSparkMax(SparkConfig motorConfig) {
    return createSparkMax(motorConfig, new SparkMaxConfig());
  }

  public static SparkMaxIO createSparkMaxFollower(SparkConfig motorConfig, int leaderCanId) {
    SparkMaxConfig config = new SparkMaxConfig();
    config.follow(leaderCanId);
    return createSparkMax(motorConfig, config);
  }

  /* CTRE */

  public static TalonFXIO createTalonFX(TalonConfig motorConfig, TalonFXConfiguration config) {
    // config.MotionMagic.MotionMagicCruiseVelocity = motorConfig.maxSpeedRotationsPerSec;
    // config.MotionMagic.MotionMagicAcceleration = motorConfig.maxAccelerationRotationsPerSecSq;
    
    PIDConfig pidConfig = motorConfig.pidConfig;
    config.Slot0.kP = pidConfig.kP;
    config.Slot0.kI = pidConfig.kI;
    config.Slot0.kD = pidConfig.kD;
    
    FFConfig ffConfig = motorConfig.ffConfig;
    config.Slot0.kS = ffConfig.kS;
    config.Slot0.kG = ffConfig.kG;
    config.Slot0.kV = ffConfig.kV;
    config.Slot0.kA = ffConfig.kA;
    
    SoftLimitConfig softLimitConfig = motorConfig.softLimitConfig;
    config.SoftwareLimitSwitch.ForwardSoftLimitEnable = softLimitConfig.forwardSoftLimitEnabled;
    config.SoftwareLimitSwitch.ForwardSoftLimitThreshold = softLimitConfig.forwardSoftLimit.in(Rotations);
    config.SoftwareLimitSwitch.ReverseSoftLimitEnable = softLimitConfig.reverseSoftLimitEnabled;
    config.SoftwareLimitSwitch.ReverseSoftLimitThreshold = softLimitConfig.reverseSoftLimit.in(Rotations);
    // config.MotorOutput.NeutralMode = motorConfig.brakeModeEnabled ? NeutralModeValue.Brake : NeutralModeValue.Coast;
    config.MotorOutput.Inverted = motorConfig.isInverted ? InvertedValue.Clockwise_Positive
        : InvertedValue.CounterClockwise_Positive;
    config.CurrentLimits.StatorCurrentLimit = motorConfig.statorCurrentLimitAmps;
    
    return new TalonFXIO(motorConfig.deviceId, new CANBus(motorConfig.canbus), config);
  }

  /** Create a TalonFX motor using its internal sensor. */
  public static TalonFXIO createTalonFX(TalonConfig talonConfig) {
    TalonFXConfiguration config = new TalonFXConfiguration();
    config.Feedback.FeedbackSensorSource = FeedbackSensorSourceValue.RotorSensor;
    config.Feedback.SensorToMechanismRatio = talonConfig.sensorToMechanismRatio;
    // TODO: make multiple TalonFX classes that handle different configs, instead of passing in TalonFXConfiguration to one
    return createTalonFX(talonConfig, config);
  }

  /** Motor and CANcoder must be on same bus. */
  public static TalonFXIO createTalonFXWithRemoteCANcoder(TalonConfig motorConfig, int cancoderId) {
    TalonFXConfiguration config = new TalonFXConfiguration();
    config.Feedback.FeedbackRemoteSensorID = cancoderId;
    config.Feedback.FeedbackSensorSource = FeedbackSensorSourceValue.RemoteCANcoder;
    return createTalonFX(motorConfig, config);
  }

  public static TalonFXSIO createTalonFXS(TalonConfig motorConfig, TalonFXSConfiguration config) {
    // config.MotionMagic.MotionMagicCruiseVelocity = motorConfig.maxSpeedRotationsPerSec;
    // config.MotionMagic.MotionMagicAcceleration = motorConfig.maxAccelerationRotationsPerSecSq;
    
    PIDConfig pidConfig = motorConfig.pidConfig;
    config.Slot0.kP = pidConfig.kP;
    config.Slot0.kI = pidConfig.kI;
    config.Slot0.kD = pidConfig.kD;
    
    FFConfig ffConfig = motorConfig.ffConfig;
    config.Slot0.kS = ffConfig.kS;
    config.Slot0.kG = ffConfig.kG;
    config.Slot0.kV = ffConfig.kV;
    config.Slot0.kA = ffConfig.kA;
    
    SoftLimitConfig softLimitConfig = motorConfig.softLimitConfig;
    config.SoftwareLimitSwitch.ForwardSoftLimitEnable = softLimitConfig.forwardSoftLimitEnabled;
    config.SoftwareLimitSwitch.ForwardSoftLimitThreshold = softLimitConfig.forwardSoftLimit.in(Rotations);
    config.SoftwareLimitSwitch.ReverseSoftLimitEnable = softLimitConfig.reverseSoftLimitEnabled;
    config.SoftwareLimitSwitch.ReverseSoftLimitThreshold = softLimitConfig.reverseSoftLimit.in(Rotations);
    config.MotorOutput.NeutralMode = motorConfig.neutralMode == NeutralMode.BRAKE ? NeutralModeValue.Brake : NeutralModeValue.Coast;
    config.MotorOutput.Inverted = motorConfig.isInverted ? InvertedValue.Clockwise_Positive
        : InvertedValue.CounterClockwise_Positive;
    config.CurrentLimits.StatorCurrentLimit = motorConfig.statorCurrentLimitAmps;
    
    return new TalonFXSIO(motorConfig.deviceId, new CANBus(motorConfig.canbus), config);
  }

  public static TalonFXSIO createTalonFXS(TalonConfig motorConfig) {
    TalonFXSConfiguration config = new TalonFXSConfiguration();
    config.ExternalFeedback.ExternalFeedbackSensorSource = ExternalFeedbackSensorSourceValue.Commutation;
    config.ExternalFeedback.SensorToMechanismRatio = motorConfig.sensorToMechanismRatio;
    return createTalonFXS(motorConfig, config);
  }

  private static TalonFXSIO createTalonFXSWithCANcoder(TalonConfig motorConfig, int cancoderId, ExternalFeedbackSensorSourceValue sensorSource) {
    TalonFXSConfiguration config = new TalonFXSConfiguration();
    config.ExternalFeedback.ExternalFeedbackSensorSource = sensorSource;
    config.ExternalFeedback.FeedbackRemoteSensorID = cancoderId;
    return createTalonFXS(motorConfig, config);
  }

  public static TalonFXSIO createTalonFXSWithRemoteCANcoder(TalonConfig motorConfig, int cancoderId) {
    return createTalonFXSWithCANcoder(motorConfig, cancoderId, ExternalFeedbackSensorSourceValue.RemoteCANcoder);
  }

  public static TalonFXSIO createTalonFXSWithFusedCANcoder(TalonConfig motorConfig, int cancoderId) {
    return createTalonFXSWithCANcoder(motorConfig, cancoderId, ExternalFeedbackSensorSourceValue.FusedCANcoder);
  }
}
