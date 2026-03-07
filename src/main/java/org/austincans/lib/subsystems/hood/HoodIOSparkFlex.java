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

package org.austincans.lib.subsystems.hood;


import com.revrobotics.spark.config.SparkFlexConfig;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.util.Units;
import java.util.ArrayList;
import java.util.List;

import org.austincans.lib.math.Angle;
import org.austincans.lib.motors.MotorFactory;
import org.austincans.lib.motors.MotorIOSparkFlex;
import org.austincans.lib.subsystems.selfcheck.HardwareFault;
import org.austincans.lib.subsystems.selfcheck.SelfChecking;
import org.austincans.lib.subsystems.selfcheck.SelfCheckingSparkFlex;

public class HoodIOSparkFlex implements HoodIO {
  private MotorIOSparkFlex motorIO;
  private SelfCheckingSparkFlex hoodMotorCheck;

  public HoodIOSparkFlex(HoodConfig hoodConfig){
    SparkFlexConfig config = new SparkFlexConfig();
    motorIO = MotorFactory.createSparkFlex(hoodConfig.deviceID(), hoodConfig.motorIsBrushless(), hoodConfig.pidffConfig(),
        hoodConfig.softLimitConfig(), hoodConfig.brakeModeEnabled(), config);
    hoodMotorCheck = new SelfCheckingSparkFlex("Hood Motor", motorIO.getMotor());
  }



  @Override
  public Angle getAngle() {
    return Angle.fromRadians(Units.rotationsToRadians(motorIO.getEncder().getPosition()));
  }

  @Override
  public Rotation2d getAngleRotation2d() {
    return Rotation2d.fromRadians(Units.rotationsToRadians(motorIO.getEncder().getPosition()));
  }

  public void setAngle(Angle setpoint) {
    motorIO.setPosition(setpoint.toRadians());
  }

  public void updateInputs(HoodIOInputs inputs) {
    inputs.hoodAngleDeg = getAngle().getRotation2d().getDegrees();
    inputs.temperatureCelsius = motorIO.getMotor().getMotorTemperature();
    inputs.motorPositionRad = Units.rotationsToRadians(motorIO.getEncder().getPosition());
    inputs.velocityRadPerSec = Units.rotationsToRadians(motorIO.getEncder().getVelocity()) / 60;
  }

  public void setAngleRotation2d(Rotation2d angle) {
    motorIO.setPosition(angle.getRadians());
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
    motorIO.setVoltage(voltage);
  }
}
