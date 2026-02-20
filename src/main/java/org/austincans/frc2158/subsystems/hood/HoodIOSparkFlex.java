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
//
// Updated on Thursday February 05, 2026 at 19:51:21 CST

package org.austincans.frc2158.subsystems.hood;

import com.google.errorprone.annotations.OverridingMethodsMustInvokeSuper;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.spark.SparkBase.ControlType;
import com.revrobotics.spark.SparkClosedLoopController;
import com.revrobotics.spark.SparkFlex;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.config.SparkFlexConfig;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.util.Units;

import java.util.ArrayList;
import java.util.List;

import org.austincans.frc2158.math.Angle;
import org.austincans.frc2158.subsystems.selfcheck.SelfChecking;
import org.austincans.frc2158.subsystems.selfcheck.SelfCheckingSparkFlex;
import org.austincans.frc2158.subsystems.selfcheck.SubsystemFault;

public class HoodIOSparkFlex implements HoodIO {
  private SparkFlex hoodMotor = new SparkFlex(HoodConstants.flexMotorID, MotorType.kBrushless);
  private SelfCheckingSparkFlex hoodMotorCheck = new SelfCheckingSparkFlex("Hood Motor", hoodMotor);
  private RelativeEncoder motorEncoder = hoodMotor.getEncoder();
  SparkClosedLoopController controller = hoodMotor.getClosedLoopController();

  SparkFlexConfig config = new SparkFlexConfig();

  public HoodIOSparkFlex() {
    config.closedLoop.p(HoodConstants.kP).i(HoodConstants.kI).d(HoodConstants.kD);
    // .outputRange(kMinOutput, kMaxOutput);
  }

  @Override
  public Angle getAngle() {
    return Angle.fromRadians(Units.rotationsToRadians(motorEncoder.getPosition()));
  }

  @Override
  public Rotation2d getAngleRotation2d() {
    return Rotation2d.fromRadians(Units.rotationsToRadians(motorEncoder.getPosition()));
  }

  public void runHoodMotor(double speed) {
    hoodMotor.set(speed);
  }

  public void setAngle(Angle setPointDegrees) {
    controller.setSetpoint(
        Units.degreesToRotations(setPointDegrees.toDegrees()), ControlType.kPosition);
  }

  public void updateInputs(HoodIOInputs inputs) {
    inputs.hoodAngleDeg = getAngle().getRotation2d().getDegrees();
    inputs.temperature = hoodMotor.getMotorTemperature();
    inputs.motorPosition = motorEncoder.getPosition();
  }

  public void setAngleRotation2d(Rotation2d angle) {
    controller.setSetpoint(Units.degreesToRotations(angle.getDegrees()), ControlType.kPosition);
  }

  @Override
  public boolean atAngle(Angle setPoint) {
    if (getAngle() == setPoint) {
      return true;
    } else {
      return false;
    }
  }

  @Override
  public List<SelfChecking> getHardware(){
    return List.of(hoodMotorCheck);
  }

  @Override
  public List<SubsystemFault> checkForFaults(){
    List<SubsystemFault> faults = new ArrayList<>();

    List<SubsystemFault> deployFault = hoodMotorCheck.checkForFaults();
    if (deployFault != null) {
      faults.addAll(deployFault);
    }

    if (faults.isEmpty()) {
      return null;
    } else {
      return faults;
    }
  }
}
