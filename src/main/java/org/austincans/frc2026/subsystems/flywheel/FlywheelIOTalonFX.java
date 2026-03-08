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

package org.austincans.frc2026.subsystems.flywheel;

import com.ctre.phoenix6.controls.VelocityVoltage;
import com.ctre.phoenix6.hardware.CANcoder;
import com.ctre.phoenix6.hardware.TalonFX;
import edu.wpi.first.units.Units;
import java.util.ArrayList;
import java.util.List;

import org.austincans.lib.subsystems.selfcheck.HardwareFault;
import org.austincans.lib.subsystems.selfcheck.SelfChecking;
import org.austincans.lib.subsystems.selfcheck.SelfCheckingTalonFXPro;

public class FlywheelIOTalonFX extends FlywheelIOReal {
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
  public List<HardwareFault> checkForFaults() {
    List<HardwareFault> faults = new ArrayList<>();
    List<HardwareFault> motorFaults = selfCheckMotor.checkForFaults();
    if (motorFaults != null) {
      faults.addAll(motorFaults);
    }
    if (faults.isEmpty()) {
      return List.of();
    } else {
      return faults;
    }
  }

  @Override
  public List<SelfChecking> getHardware() {
    return List.of(selfCheckMotor);
  }
}
