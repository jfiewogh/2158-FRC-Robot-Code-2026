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

package org.austincans.frc2026.subsystems.intake;

import java.util.List;
import org.austincans.frc2026.subsystems.selfcheck.HardwareFault;
import org.austincans.frc2026.subsystems.selfcheck.SelfChecking;
import org.littletonrobotics.junction.AutoLog;

/** Add your docs here. */
public interface IntakeIO {
  @AutoLog
  public class IntakeIOInputs {
    public boolean isDeployed = false;
    public double deployMotorVoltage = 0;
    public double deployPositionRad = 0;
    public double deployTemperatureC = 0;
    public double intakeMotorVoltage = 0;
    public double intakeVelocityRadPerSec = 0;
    public double intakeTemperatureC = 0;
  }

  public class IntakeIOOutputs {
    public double velocity = 0;
    public double voltage = 0;
    public IntakeIOOutputsMode mode = IntakeIOOutputsMode.VELOCITY;
  }

  public static enum IntakeIOOutputsMode{
    VELOCITY,
    VOLTAGE
  }

  public void updateInputs(IntakeIOInputs inputs);

  public void applyOutputs(IntakeIOOutputs outputs);

  // rpm
  public void setRelativeVelocity(double velocity);

  public void setVoltage(double voltage);

  public default List<HardwareFault> checkForFaults() {
    return List.of();
  }

  public default List<SelfChecking> getHardware() {
    return List.of();
  }
}
