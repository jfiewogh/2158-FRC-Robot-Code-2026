package org.austincans.lib.io.motors;

import com.ctre.phoenix6.CANBus;
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
  public static TalonFXIO createTalonFX(int deviceId, CANBus canBus) {
    return new TalonFXIO(deviceId, canBus);
  }

  /** Create a TalonFXS motor */
  public static TalonFXSIO createTalonFXS(int deviceId, CANBus canBus) {
    return new TalonFXSIO(deviceId, canBus);
  }
}
