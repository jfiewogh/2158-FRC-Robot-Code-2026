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

package org.austincans.frc2026.subsystems.hood;

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

import org.austincans.lib.math.Angle;
import org.austincans.lib.subsystems.selfcheck.HardwareFault;
import org.austincans.lib.subsystems.selfcheck.SelfChecking;
import org.austincans.lib.subsystems.selfcheck.SelfCheckingSparkFlex;

public class HoodIOSparkFlex implements HoodIO {
  private SparkFlex motor = new SparkFlex(HoodConstants.flexMotorID, MotorType.kBrushless);
  private SelfCheckingSparkFlex hoodMotorCheck = new SelfCheckingSparkFlex("Hood Motor", motor);
  private RelativeEncoder motorEncoder = motor.getEncoder();
  SparkClosedLoopController controller = motor.getClosedLoopController();

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

  public void setAngle(Angle setpoint) {
    controller.setSetpoint(setpoint.toRotations(), ControlType.kPosition);
  }

  public void updateInputs(HoodIOInputs inputs) {
    inputs.hoodAngleDeg = getAngle().getRotation2d().getDegrees();
    inputs.temperatureCelsius = motor.getMotorTemperature();
    inputs.motorPositionRad = Units.rotationsToRadians(motorEncoder.getPosition());
    inputs.velocityRadPerSec = Units.rotationsToRadians(motorEncoder.getVelocity()) / 60;
  }

  public void setAngleRotation2d(Rotation2d angle) {
    controller.setSetpoint(Units.degreesToRotations(angle.getDegrees()), ControlType.kPosition);
  }

  @Override
  public boolean atAngle(Angle setpoint) {
    return getAngle() == setpoint;
  }

  @Override
  public List<SelfChecking> getHardware() {
    return List.of(hoodMotorCheck);
  }

  @Override
  public List<HardwareFault> checkForFaults() {
    List<HardwareFault> faults = new ArrayList<>();

    List<HardwareFault> deployFault = hoodMotorCheck.checkForFaults();
    if (deployFault != null) {
      faults.addAll(deployFault);
    }

    if (faults.isEmpty()) {
      return null;
    } else {
      return faults;
    }
  }

  @Override
  public void setVoltage(double voltage) {
    motor.setVoltage(voltage);
  }
}
