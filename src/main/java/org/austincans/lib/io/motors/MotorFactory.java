package org.austincans.lib.io.motors;

/** 
 * This class should be used to create the MotorIO instances. 
 */
public class MotorFactory {
  /** Return new instance of SparkFlexIO based on SparkConfig */
  public static SparkFlexIO createSparkFlex(SparkConfig config) {
    return new SparkFlexIO(config);
  }

  /** Return new instance of SparkMaxIO based on SparkConfig */
  public static SparkMaxIO createSparkMax(SparkConfig config) {
    return new SparkMaxIO(config);
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
