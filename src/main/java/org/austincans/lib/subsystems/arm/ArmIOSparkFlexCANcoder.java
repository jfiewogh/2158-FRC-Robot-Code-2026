// Copyright (c) 2026 the ausTIN CANs
// https://github.com/AusTINCANsProgrammingTeam
//
// oooooooooooo ooooooooo.     .oooooo.        ooooooooo.              .o8                     .   
// `888'     `8 `888   `Y88.  d8P'  `Y8b       `888   `Y88.           "888                   .o8   
//  888          888   .d88' 888                888   .d88'  .ooooo.   888oooo.   .ooooo.  .o888oo 
//  888oooo8     888ooo88P'  888                888ooo88P'  d88' `88b  d88' `88b d88' `88b   888   
//  888    "     888`88b.    888                888`88b.    888   888  888   888 888   888   888   
//  888          888  `88b.  `88b    ooo        888  `88b.  888   888  888   888 888   888   888 . 
// o888o        o888o  o888o  `Y8bood8P'       o888o  o888o `Y8bod8P'  `Y8bod8P' `Y8bod8P'   "888" 
//
// Use of this source code is governed by a BSD
// license that can be found in the LICENSE file
// at the root directory of this project.

package org.austincans.lib.subsystems.arm;

import com.revrobotics.AbsoluteEncoder;
import com.revrobotics.PersistMode;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.ResetMode;
import com.revrobotics.spark.SparkBase.ControlType;
import com.revrobotics.spark.SparkClosedLoopController;
import com.revrobotics.spark.SparkFlex;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.config.ClosedLoopConfig;
import com.revrobotics.spark.config.SoftLimitConfig;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;
import com.revrobotics.spark.config.SparkFlexConfig;
import java.util.List;
import org.austincans.lib.util.KalmanFilter1D;
import org.austincans.lib.encoders.AbsoluteEncoderFactory;
import org.austincans.lib.encoders.CANcoderIO;
import org.austincans.lib.motors.MotorConfig;
import org.austincans.lib.motors.MotorFactory;
import org.austincans.lib.motors.MotorIOSparkFlex;
import org.austincans.lib.subsystems.selfcheck.HardwareFault;
import org.austincans.lib.subsystems.selfcheck.SelfChecking;
import org.austincans.lib.subsystems.selfcheck.SelfCheckingSparkFlex;

public class ArmIOSparkFlexCANcoder extends ArmIOReal<MotorIOSparkFlex, CANcoderIO> {
  private final SelfCheckingSparkFlex selfCheckingMotor;

  @SuppressWarnings("unused")
  private KalmanFilter1D filter1d = new KalmanFilter1D(0, 0, 0);

  public ArmIOSparkFlexCANcoder(ArmConfig armConfig) {
    super(
      MotorFactory.createSparkFlex(armConfig.motorConfig()), 
      AbsoluteEncoderFactory.createCANcoder(armConfig.absoluteEncoderConfig()));
    motorIO.setEncoderPosition(absoluteEncoderIO.getPosition());
    selfCheckingMotor = new SelfCheckingSparkFlex("Arm", motorIO.getMotor());
    // Configure encoder
    motorIO.setEncoderPosition(0);
  }

  @Override
  public void setVoltage(double voltage) {
    motorIO.setVoltage(voltage);
  }

  public void setPosition(double positionRad) {
    motorIO.setPosition(positionRad);
  }

  public void setVelocity(double velocityRadPerSec) {
    motorIO.setVelocity(velocityRadPerSec);
  }

  public void updateInputs(ArmIOInputs inputs) {
    inputs.motorInputs = motorInputs;
    inputs.positionRad = motorInputs.positionRad;
    inputs.velocityRadPerSec = motorInputs.velocityRadPerSec;
  }

  public void stopMotor() {
    setVoltage(0);
  }

  @Override
  public List<HardwareFault> checkForFaults() {
    List<HardwareFault> faults = selfCheckingMotor.checkForFaults();
    if (faults.isEmpty()) {
      return null;
    } else {
      return faults;
    }
  }

  @Override
  public List<SelfChecking> getHardware() {
    return List.of(selfCheckingMotor);
  }
}
