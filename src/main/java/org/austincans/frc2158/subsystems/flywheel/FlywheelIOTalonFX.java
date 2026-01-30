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
// Updated on Thursday January 22, 2026 at 19:06:38 CST

package org.austincans.frc2158.subsystems.flywheel;

import com.ctre.phoenix6.controls.VelocityVoltage;
import com.ctre.phoenix6.hardware.CANcoder;
import com.ctre.phoenix6.hardware.TalonFX;
import edu.wpi.first.units.Units;
import java.util.ArrayList;
import java.util.List;
import org.austincans.frc2158.subsystems.selfcheck.SelfCheckingTalonFXPro;
import org.austincans.frc2158.subsystems.selfcheck.SubsystemFault;

public class FlywheelIOTalonFX implements FlywheelIO {
  private final TalonFX motor = new TalonFX(FlywheelConstants.motorId, FlywheelConstants.canBus);
  private final SelfCheckingTalonFXPro selfCheckMotor =
      new SelfCheckingTalonFXPro("flywheel", motor);

  private final CANcoder encoder =
      new CANcoder(FlywheelConstants.encoderId, FlywheelConstants.canBus);

  public FlywheelIOTalonFX() {}

  @Override
  public void updateInputs(FlywheelIOInputs inputs) {
    inputs.appliedVolts = motor.getMotorVoltage().getValue().in(Units.Volts);
    inputs.velocityRPM = encoder.getVelocity().getValue().in(Units.RPM);
    inputs.temperatureC = motor.getDeviceTemp().getValue().in(Units.Celsius);
    inputs.currentAmps =motor.getStatorCurrent().getValue().in(Units.Amps);
  }

  @Override
  public void runVoltage(double volts) {
    motor.setVoltage(volts);
  }

  @Override
  public void runVelocity(double velocityRPM) {
    motor.setControl(new VelocityVoltage(velocityRPM).withVelocity(velocityRPM / 60.0));
  }

  @Override
  public List<SubsystemFault> checkForFaults() {
    List<SubsystemFault> faults = new ArrayList<>();
    List<SubsystemFault> motorFaults = selfCheckMotor.checkForFaults();
    if (motorFaults != null) {
      faults.addAll(motorFaults);
    }
    if (faults.isEmpty()) {
      return null;
    } else {
      return faults;
    }
  }
}
