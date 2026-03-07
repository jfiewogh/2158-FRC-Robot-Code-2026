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

import static edu.wpi.first.units.Units.Radians;
import static edu.wpi.first.units.Units.RadiansPerSecond;

import com.ctre.phoenix6.configs.TalonFXSConfiguration;
import com.ctre.phoenix6.controls.DutyCycleOut;
import com.ctre.phoenix6.hardware.TalonFXS;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.util.Units;
import java.util.ArrayList;
import java.util.List;

import org.austincans.lib.math.Angle;
import org.austincans.lib.subsystems.selfcheck.HardwareFault;
import org.austincans.lib.subsystems.selfcheck.SelfChecking;
import org.austincans.lib.subsystems.selfcheck.SelfCheckingTalonFXS;

public class HoodIOTalonFXS implements HoodIO {
  private TalonFXS motor = new TalonFXS(HoodConstants.talonMotorID);
  private SelfCheckingTalonFXS hoodMotorCheck = new SelfCheckingTalonFXS("Hood Motor", motor);
  private DutyCycleOut dc = new DutyCycleOut(0);
  TalonFXSConfiguration configs = new TalonFXSConfiguration();

  public HoodIOTalonFXS() {
    configs.Slot0.kP = HoodConstants.kP;
    configs.Slot0.kI = HoodConstants.kI;
    configs.Slot0.kD = HoodConstants.kD;
    configs.Slot0.kV = HoodConstants.kV;
  }

  @Override
  public Angle getAngle() {
    return Angle.fromDegrees(motor.getPosition().getValue().in(null));
  }

  public void setAngle(Angle setpoint) {
    motor.setControl(dc.withOutput(setpoint.toRotations()));
  }

  @Override
  public void setAngleRotation2d(Rotation2d angle) {
    motor.setControl(dc.withOutput(Units.degreesToRotations(angle.getDegrees())));
  }

  @Override
  public Rotation2d getAngleRotation2d() {
    return Rotation2d.fromRadians(
        Units.rotationsToRadians(motor.getPosition().getValue().in(null)));
  }

  @Override
  public boolean atAngle(Angle setPoint) {
    if (getAngle() == setPoint) {
      return true;
    } else {
      return false;
    }
  }

  public void updateInputs(HoodIOInputs inputs) {
    inputs.hoodAngleDeg = getAngle().getRotation2d().getDegrees();
    inputs.temperatureCelsius = motor.getDeviceTemp().getValueAsDouble();
    inputs.motorPositionRad = motor.getPosition().getValue().in(Radians);
    inputs.velocityRadPerSec = motor.getVelocity().getValue().in(RadiansPerSecond);
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
