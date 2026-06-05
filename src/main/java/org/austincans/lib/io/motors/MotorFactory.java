package org.austincans.lib.io.motors;

import com.revrobotics.spark.SparkLowLevel.MotorType;

/** 
 * This class should be used to create the MotorIO instances. 
 */
public class MotorFactory {
  /** Return new instance of SparkFlexIO based on SparkConfig */
  public static SparkFlexIO createSparkFlex(int deviceId, MotorType motorType) {
    return new SparkFlexIO(deviceId, motorType);
  }

  /** Return new instance of SparkMaxIO based on SparkConfig */
  public static SparkMaxIO createSparkMax(int deviceId, MotorType motorType) {
    return new SparkMaxIO(deviceId, motorType);
  }

  /** Create a TalonFX motor */
  public static TalonFXIO createTalonFX(TalonFXConfig config) {
    return createTalonFX(config);
  }

  /** Create a TalonFXS motor */
  public static TalonFXSIO createTalonFXS(TalonFXSConfig config) {
    return createTalonFXS(config);
  }
}
