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

import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import org.austincans.frc2026.controllers.LaserCAN;
import org.austincans.frc2026.subsystems.intake.deploy.DeployIO;
import org.austincans.frc2026.subsystems.selfcheck.AdvancedSubsystem;
import org.littletonrobotics.junction.AutoLogOutput;
import org.littletonrobotics.junction.Logger;

public class Intake extends AdvancedSubsystem {
  private final IntakeIO intakeIO;
  private final IntakeIOInputsAutoLogged inputs = new IntakeIOInputsAutoLogged();
  private final DeployIO deployIO;

  private final LaserCAN laserCAN = new LaserCAN(IntakeConstants.laserCANID);
  private final DigitalInput magLimitSwitch =
      new DigitalInput(IntakeConstants.magLimitSwitchChannel);

  @AutoLogOutput public static boolean deployed = false;

  public Intake(IntakeIO intakeIO, DeployIO deployIO) {
    this.intakeIO = intakeIO;
    this.deployIO = deployIO;
    try {
      this.intakeIO
          .getHardware()
          .forEach(
              e -> {
                if (e == null) {
                  return;
                }
                this.registerHardware(e);
              });
    } catch (NullPointerException e) {
      System.out.println("Null Hardware intake");
    }
    try {
      this.deployIO
          .getHardware()
          .forEach(
              e -> {
                if (e == null) {
                  return;
                }
                this.registerHardware(e);
              });
    } catch (NullPointerException e) {
      System.out.println("Null Hardware deploy");
    }
    this.registerCheckCommand(systemCheckCommand());
  }

  public boolean intakeDeployed() {
    return magLimitSwitch.get();
  }

  public void setRelativeVelocity(double velocity) {
    intakeIO.setRelativeVelocity(velocity);
  }

  public void setVoltage(double voltage) {
    intakeIO.setVoltage(voltage);
  }

  public void deploy() {
    deployIO.deploy();
  }

  public void retract() {
    deployIO.retract();
  }

  public void stopDeploy() {
    deployIO.stopMotors();
  }

  public void runDeployVelocity(double velocityRotationsPerSec) {
    deployIO.setVelocity(velocityRotationsPerSec);
  }

  public double getVelocityRadPerSec() {
    return inputs.intakeVelocityRadPerSec;
  }

  @Override
  public void periodic() {
    intakeIO.updateInputs(inputs);
    deployIO.updateInputs(inputs);

    try {
      intakeIO
          .checkForFaults()
          .forEach(
              e -> {
                if (e == null) {
                  return;
                }
                this.addHardwareFault(e);
              });
    } catch (NullPointerException e) {
      System.out.println("Null Faults intake");
    }
    try {
      deployIO.updateInputs(inputs);
      deployIO
          .checkForFaults()
          .forEach(
              e -> {
                if (e == null) {
                  return;
                }
                this.addHardwareFault(e);
              });
    } catch (NullPointerException e) {
      System.out.println("Null pointer deploy");
    }
    inputs.isDeployed = intakeDeployed();

    Logger.processInputs("Intake", inputs);
  }

  public Command systemCheckCommand() {
    return Commands.runOnce(() -> {});
  }

  public Command deployCommand() {
    return Commands.runOnce(
        () -> {
          deployIO.deploy();
        },
        this);
  }

  public Command retractCommand() {
    return Commands.runOnce(
        () -> {
          intakeIO.setRelativeVelocity(0);
          deployIO.retract();
        },
        this);
  }
}
