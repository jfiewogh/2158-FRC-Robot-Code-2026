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

package org.austincans.frc2026.subsystems.battery;

import edu.wpi.first.wpilibj.Alert;
import edu.wpi.first.wpilibj.Alert.AlertType;
import edu.wpi.first.wpilibj.PowerDistribution;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.simulation.DriverStationSim;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import org.austincans.frc2026.Robot;
import org.littletonrobotics.junction.Logger;

public class BatterySubsystem extends SubsystemBase {
  private double voltage;
  private double current;
  private double simVoltage;
  private double loggedTimer;
  private double timerHighCurrent;

  private Timer timer = new Timer();
  private Timer currentTimer = new Timer();
  private PowerDistribution powerDistribution = new PowerDistribution();

  private Alert lowVoltageWarning = new Alert("Change the Battery Soon! (VY)", AlertType.kWarning);
  private Alert lowVoltageError = new Alert("Change the Battery Soon! (VY)", AlertType.kError);
  private Alert checkBatteryHealth =
      new Alert("Battery Current Low, Check Battery", AlertType.kError);

  public BatterySubsystem() {
    Logger.recordOutput("Battery/Voltage", voltage);
    Logger.recordOutput("Battery/Current", current);
    Logger.recordOutput("Battery/simVoltage", simVoltage);
    Logger.recordOutput("Battery/Timer", loggedTimer);
    timer.start();
    DriverStationSim.setSendError(true);
  }

  public void periodic() {
    voltage = getVoltage();
    current = getInputCurrent();
    simVoltage = powerDistribution.getVoltage();
    checkCurrent();
    checkVoltage();
  }

  public double getVoltage() {
    return powerDistribution.getVoltage();
  }

  public double getInputCurrent() {
    if (Robot.isSimulation()) {
      return powerDistribution.getCurrent(1);
    } else {
      return powerDistribution.getTotalCurrent();
    }
  }

  public void checkCurrent() {
    if (getInputCurrent() < BatteryConstants.highBatteryCurrentThreshold) {
      checkBatteryHealth.set(true);
    }
  }

  public void checkVoltage() {
    if (getVoltage() < BatteryConstants.minVoltageRed) {
      lowVoltageWarning.set(true);
    } else if (getVoltage() < BatteryConstants.minVoltageYellow) {
      lowVoltageError.set(true);
    } else {
      lowVoltageWarning.set(false);
      lowVoltageError.set(false);
    }
  }
}
