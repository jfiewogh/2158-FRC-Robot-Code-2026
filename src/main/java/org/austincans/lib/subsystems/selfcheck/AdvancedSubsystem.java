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

package org.austincans.lib.subsystems.selfcheck;

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
import org.austincans.lib.controllers.REVDigitController;
import org.littletonrobotics.junction.Logger;

/*
 * Links to complete selfchecking
 * https://github.com/3015RangerRobotics/2024Public/tree/main/RobotCode2024/src/main/java/frc/lib/subsystem
 * https://github.com/3015RangerRobotics/2024Public/tree/main/RobotCode2024/src/main/java/frc/robot/subsystems
 * https://github.com/3015RangerRobotics/2024Public/blob/main/RobotCode2024/src/main/java/frc/robot/subsystems/swerve/Swerve.java
 * https://github.com/3015RangerRobotics/2024Public/blob/main/RobotCode2024/src/main/java/frc/robot/subsystems/swerve/SwerveIOPhoenix.java
 */

public abstract class AdvancedSubsystem extends SubsystemBase {
  public enum SystemStatus {
    OK,
    WARNING,
    ERROR
  }

  public enum FaultCodes {}

  private static final List<SubsystemFault> subsystemFaults = new ArrayList<>();
  private final List<HardwareFault> hardwareFaults = new ArrayList<>();
  private final List<SelfChecking> hardware = new ArrayList<>();
  private static final List<Command> checkCommands = new ArrayList<>();
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
    Logger.recordOutput(statusTable + "/Status", status.name());
    Logger.recordOutput(statusTable + "/SystemOK", status == SystemStatus.OK);

    String[] faultStrings = new String[this.hardwareFaults.size()];
    for (int i = 0; i < this.hardwareFaults.size(); i++) {
      HardwareFault fault = this.hardwareFaults.get(i);
      faultStrings[i] = String.format("[%.2f] %s", fault.timestamp, fault.description);
    }
    Logger.recordOutput(statusTable + "/Faults", faultStrings);

    if (faultStrings.length > 0) {
      Logger.recordOutput(statusTable + "/LastFault", faultStrings[faultStrings.length - 1]);
    } else {
      Logger.recordOutput(statusTable + "/LastFault", "");
    }
  }

  protected void reportPowerUsage(String device, double amps, double volts) {
    //    BatteryUsage.reportUsage(this.getName(), device, amps, volts);
  }

  protected void addSubsystemFault(SubsystemFault fault) {
    if (!AdvancedSubsystem.subsystemFaults.contains(fault)) {
      AdvancedSubsystem.subsystemFaults.add(fault);
    }
  }

  protected void addHardwareFault(HardwareFault fault) {
    if (!this.hardwareFaults.contains(fault)) {
      this.hardwareFaults.add(fault);
    }
  }

  protected void addSubsystemFault(String code, String description, boolean isWarning) {
    this.addSubsystemFault(new SubsystemFault(code, description, isWarning));
  }

  protected void addSubsystemFault(String code, String description) {
    this.addSubsystemFault(code, description, false);
  }

  protected void addHardwareFault(
      String code, String description, boolean isWarning, boolean isSticky) {
    this.addHardwareFault(new HardwareFault(code, description, isWarning, isSticky));
  }

  protected void addHardwareFault(String code, String description, boolean isWarning) {
    this.addHardwareFault(new HardwareFault(code, description, isWarning, false));
  }

  protected void addHardwareFault(String code, String description) {
    this.addHardwareFault(code, description, false);
  }

  public List<SubsystemFault> getSubsystemFaults() {
    return AdvancedSubsystem.subsystemFaults;
  }

  public void clearFaults() {
    AdvancedSubsystem.subsystemFaults.clear();
    this.hardwareFaults.clear();
  }

  public SystemStatus getSystemStatus() {
    SystemStatus worstStatus = SystemStatus.OK;

    for (HardwareFault f : this.hardwareFaults) {
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

    for (SubsystemFault f : AdvancedSubsystem.subsystemFaults) {
      if (f.timestamp > Timer.getFPGATimestamp() - 10) {
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

  public void registerHardware(SelfChecking item) {
    hardware.add(item);
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

  public void registerCheckCommand(Command command) {
    checkCommands.add(command);
  }

  public static Command sequentialCheck() {
    return Commands.sequence(checkCommands.toArray(Command[]::new))
        .onlyWhile(() -> subsystemFaults.size() == 0)
        .handleInterrupt(
            () ->
                REVDigitController.getInstance()
                    .writeDigits(subsystemFaults.get(0).description.toString()));
  }

  // Method to check for faults while the robot is operating normally
  private void checkForFaults() {
    if (checkErrors) {
      for (SelfChecking device : hardware) {
        for (HardwareFault fault : device.checkForFaults()) {
          addHardwareFault(fault);
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
