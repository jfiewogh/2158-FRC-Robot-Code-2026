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

import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import lombok.Getter;
import lombok.experimental.Accessors;
import org.austincans.frc2026.Robot;
import org.austincans.frc2026.subsystems.flywheel.FlywheelIO.FlywheelIOOutputMode;
import org.austincans.frc2026.subsystems.flywheel.FlywheelIO.FlywheelIOOutputs;
import org.austincans.frc2026.subsystems.selfcheck.AdvancedSubsystem;
import org.austincans.frc2026.util.EqualsUtil;
import org.littletonrobotics.junction.AutoLogOutput;
import org.littletonrobotics.junction.Logger;

public class Flywheel extends AdvancedSubsystem {
  private final FlywheelIO io;
  private final FlywheelIOInputsAutoLogged inputs = new FlywheelIOInputsAutoLogged();
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

    double velocityRadPerSec = Units.rotationsPerMinuteToRadiansPerSecond(inputs.velocityRPM);
    double setpointRadPerSec = Units.rotationsPerMinuteToRadiansPerSecond(outputs.velocityRPM);
    atGoal = EqualsUtil.epsilonEquals(setpointRadPerSec, velocityRadPerSec, 2.0);
    Logger.recordOutput("Flywheel/Setpoint", setpointRadPerSec);
    Logger.recordOutput("Flywheel/Goal", velocityRadPerSec);

    Robot.batteryLogger.reportCurrentUsage(
        "Flywheel", false, inputs.currentAmps);
  }

  @Override
  public void periodicAfterScheduler() {
    io.applyOutputs(outputs);
    // LoggedTracer.record("Flywheel/AfterScheduler");
  }

  /** Run closed loop at the specified velocity. */
  public void setVelocity(double velocityRPM) {
    outputs.mode = FlywheelIOOutputMode.VELOCITY;
    outputs.velocityRPM = velocityRPM;
  }

  public void stop() {
    setVelocity(0);
  }

  public double getRPM() {
    return inputs.velocityRPM;
  }

  public boolean atSetpoint() {
    boolean a =
        outputs.mode == FlywheelIOOutputMode.VELOCITY
            && Math.abs(getRPM() - outputs.velocityRPM) < outputs.velocityRPM * 0.1;
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
