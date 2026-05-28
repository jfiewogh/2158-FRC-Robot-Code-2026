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
