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

package org.austincans.lib.subsystems.flywheel;

import edu.wpi.first.math.util.Units;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import lombok.Getter;
import lombok.experimental.Accessors;

import static edu.wpi.first.units.Units.RPM;
import static edu.wpi.first.units.Units.Volts;

import org.austincans.frc2026.Robot;
import org.austincans.lib.motors.MotorIO.MotorIOOutputMode;
import org.austincans.lib.subsystems.flywheel.FlywheelIO.FlywheelIOInputs;
import org.austincans.lib.subsystems.flywheel.FlywheelIO.FlywheelIOOutputs;
import org.austincans.lib.subsystems.selfcheck.AdvancedSubsystem;
import org.austincans.lib.util.EqualsUtil;
import org.littletonrobotics.junction.AutoLogOutput;
import org.littletonrobotics.junction.Logger;

public class Flywheel extends AdvancedSubsystem {
  private final FlywheelIO io;
  private final FlywheelIOInputs inputs = new FlywheelIOInputs();
  private final FlywheelIOOutputs outputs = new FlywheelIOOutputs();

  @Getter
  @Accessors(fluent = true)
  @AutoLogOutput(key = "Flywheel/AtGoal")
  private boolean atGoal = false;

  /** Creates a new Flywheel. */
  public Flywheel(FlywheelIO io) {
    this.io = io;
    io.registerSelfCheckHardware(this);
    registerCheckCommand(systemCheckCommand());
  }

  @Override
  public void periodic() {
    io.updateInputs(inputs);
    Logger.processInputs("Flywheel", inputs);

    double velocityRadPerSec = Units.rotationsPerMinuteToRadiansPerSecond(inputs.motorInputs.getVelocity());
    double setpointRadPerSec = Units.rotationsPerMinuteToRadiansPerSecond(outputs.motorOutputs.getVeolcity());
    atGoal = EqualsUtil.epsilonEquals(setpointRadPerSec, velocityRadPerSec, 2.0);
    Logger.recordOutput("Flywheel/Setpoint", setpointRadPerSec);
    Logger.recordOutput("Flywheel/Goal", velocityRadPerSec);

    Robot.batteryLogger.reportCurrentUsage("Flywheel", false, inputs.motorInputs.supplyCurrentAmps);
  }

  @Override
  public void periodicAfterScheduler() {
    io.applyOutputs(outputs);
    // LoggedTracer.record("Flywheel/AfterScheduler");
  }

  /** Run closed loop at the specified velocity. */
  public void setVelocity(double velocityRPM) {
    outputs.motorOutputs.setVelocity(RPM.of(velocityRPM));
  }

  public void stop() {
    outputs.motorOutputs.setVoltage(Volts.of(0.0));
  }

  public double getRPM() {
    return inputs.motorInputs.velocity.baseUnitMagnitude();
  }

  public boolean atSetpoint() {
    boolean a =
        outputs.motorOutputs.mode == MotorIOOutputMode.VELOCITY
            && Math.abs(getRPM() - outputs.motorOutputs.getVeolcity()) < outputs.motorOutputs.getVeolcity() * 0.1;
    Logger.recordOutput("flywheel at setpoint", a);
    return a;
  }

  @Override
  public Command systemCheckCommand() {
    return Commands.sequence(
        Commands.run(() -> setVelocity(100), this).withTimeout(5),
        Commands.waitSeconds(2),
        Commands.runOnce(
            () -> {
              if (getRPM() < 0) {
                addSubsystemFault(
                    "FLY", "[System Check] Flywheel did not reach desired velocity RPM");
              }
            },
            this),
        Commands.runOnce(this::stop, this));
  }
}