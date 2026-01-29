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
// Updated on Wednesday January 28, 2026 at 19:56:47 CST

package org.austincans.frc2158.subsystems.selfcheck;

import com.ctre.phoenix6.hardware.CANcoder;
import com.revrobotics.spark.SparkFlex;
import com.revrobotics.spark.SparkMax;
import edu.wpi.first.wpilibj.RobotBase;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.motorcontrol.PWMMotorController;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import java.util.ArrayList;
import java.util.List;

public abstract class AdvancedSubsystem extends SubsystemBase {
  public enum SystemStatus {
    OK,
    WARNING,
    ERROR
  }

  private final List<SubsystemFault> faults = new ArrayList<>();
  private final List<SelfChecking> hardware = new ArrayList<>();
  private final String statusTable;
  private final boolean checkErrors;

  public AdvancedSubsystem() {
    this.statusTable = "SystemStatus/" + this.getName();
    Command systemCheck = getSystemCheckCommand();
    systemCheck.setName(getName() + "Check");
    SmartDashboard.putData(statusTable + "/SystemCheck", systemCheck);
    SmartDashboard.putBoolean(statusTable + "/CheckRan", false);
    checkErrors = RobotBase.isReal();

    setupCallbacks();
  }

  public AdvancedSubsystem(String name) {
    this.setName(name);
    this.statusTable = "SystemStatus/" + name;
    Command systemCheck = getSystemCheckCommand();
    systemCheck.setName(getName() + "Check");
    SmartDashboard.putData(statusTable + "/SystemCheck", systemCheck);
    SmartDashboard.putBoolean(statusTable + "/CheckRan", false);
    checkErrors = RobotBase.isReal();

    setupCallbacks();
  }

  public Command getSystemCheckCommand() {
    return Commands.sequence(
        Commands.runOnce(
            () -> {
              SmartDashboard.putBoolean(statusTable + "/CheckRan", false);
              clearFaults();
              publishStatus();
            }),
        systemCheckCommand(),
        Commands.runOnce(
            () -> {
              publishStatus();
              SmartDashboard.putBoolean(statusTable + "/CheckRan", true);
            }));
  }

  private void setupCallbacks() {
    // Robot.addPeriodicCallback(this::checkForFaults, 0.25);
    // Robot.addPeriodicCallback(this::publishStatus, 1.0);
  }

  private void publishStatus() {
    SystemStatus status = getSystemStatus();
    SmartDashboard.putString(statusTable + "/Status", status.name());
    SmartDashboard.putBoolean(statusTable + "/SystemOK", status == SystemStatus.OK);

    String[] faultStrings = new String[this.faults.size()];
    for (int i = 0; i < this.faults.size(); i++) {
      SubsystemFault fault = this.faults.get(i);
      faultStrings[i] = String.format("[%.2f] %s", fault.timestamp, fault.description);
    }
    SmartDashboard.putStringArray(statusTable + "/Faults", faultStrings);

    if (faultStrings.length > 0) {
      SmartDashboard.putString(statusTable + "/LastFault", faultStrings[faultStrings.length - 1]);
    } else {
      SmartDashboard.putString(statusTable + "/LastFault", "");
    }
  }

  protected void reportPowerUsage(String device, double amps, double volts) {
    //    BatteryUsage.reportUsage(this.getName(), device, amps, volts);
  }

  protected void addFault(SubsystemFault fault) {
    if (!this.faults.contains(fault)) {
      this.faults.add(fault);
    }
  }

  protected void addFault(String description, boolean isWarning) {
    this.addFault(new SubsystemFault(description, isWarning));
  }

  protected void addFault(String description, boolean isWarning, boolean sticky) {
    this.addFault(new SubsystemFault(description, isWarning, sticky));
  }

  protected void addFault(String description) {
    this.addFault(description, false);
  }

  public List<SubsystemFault> getFaults() {
    return this.faults;
  }

  public void clearFaults() {
    this.faults.clear();
  }

  public SystemStatus getSystemStatus() {
    SystemStatus worstStatus = SystemStatus.OK;

    for (SubsystemFault f : this.faults) {
      if (f.sticky || f.timestamp > Timer.getFPGATimestamp() - 10) {
        if (f.isWarning) {
          if (worstStatus != SystemStatus.ERROR) {
            worstStatus = SystemStatus.WARNING;
          }
        } else {
          worstStatus = SystemStatus.ERROR;
        }
      }
    }
    return worstStatus;
  }

  public void registerHardware(String label, com.ctre.phoenix6.hardware.TalonFX talon) {
    hardware.add(new SelfCheckingTalonFXPro(label, talon));
  }

  public void registerHardware(String label, com.ctre.phoenix6.hardware.TalonFXS talon) {
    hardware.add(new SelfCheckingTalonFXS(label, talon));
  }

  public void registerHardware(String label, PWMMotorController pwmMotor) {
    hardware.add(new SelfCheckingPWMMotor(label, pwmMotor));
  }

  public void registerHardware(String label, SparkMax spark) {
    hardware.add(new SelfCheckingSparkMax(label, spark));
  }

  public void registerHardware(String label, SparkFlex spark) {
    hardware.add(new SelfCheckingSparkFlex(label, spark));
  }

  public void registerHardware(String label, com.ctre.phoenix6.hardware.Pigeon2 pigeon2) {
    hardware.add(new SelfCheckingPigeon2(label, pigeon2));
  }

  public void registerHardware(String label, CANcoder canCoder) {
    hardware.add(new SelfCheckingCANCoderPro(label, canCoder));
  }

  // Command to run a full systems check
  protected abstract Command systemCheckCommand();

  // Method to check for faults while the robot is operating normally
  private void checkForFaults() {
    if (checkErrors) {
      for (SelfChecking device : hardware) {
        for (SubsystemFault fault : device.checkForFaults()) {
          addFault(fault);
        }
      }
    }
  }

  @Override
  public void periodic() {
    checkForFaults();
    publishStatus();
  }
}
