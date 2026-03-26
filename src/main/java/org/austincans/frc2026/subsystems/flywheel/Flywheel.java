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

import com.revrobotics.PersistMode;
import com.revrobotics.ResetMode;
import com.revrobotics.spark.ClosedLoopSlot;
import com.revrobotics.spark.SparkBase.ControlType;
import com.revrobotics.spark.SparkClosedLoopController;
import com.revrobotics.spark.SparkFlex;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.config.SparkFlexConfig;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import org.austincans.frc2026.subsystems.intake.IntakeConstants;
import org.austincans.frc2026.subsystems.selfcheck.AdvancedSubsystem;
import org.littletonrobotics.junction.Logger;

// NOTE: Sometimes flywheel run velocity does not work
/*
 * In statistics and control theory, Kalman filtering (also known as linear quadratic estimation)
 * is an algorithm that uses a series of measurements observed over time, including statistical
 * noise and other inaccuracies, to produce estimates of unknown variables that tend to be more
 * accurate than those based on a single measurement, by estimating a joint probability
 * distribution over the variables for each time-step.
 * The filter is constructed as a mean squared error minimiser, but an alternative derivation of
 * the filter is also provided showing how the filter relates to maximum likelihood statistics.
 * https://docs.wpilib.org/en/stable/docs/software/advanced-controls/state-space/state-space-observers.html#kalman-filters
 */

public class Flywheel extends AdvancedSubsystem {
  private final FlywheelIO io;
  private final FlywheelIOInputsAutoLogged inputs = new FlywheelIOInputsAutoLogged();
  public static boolean isShooting = false;
  public static boolean isReadyToShoot = false;
  private final SparkFlex indexer =
      new SparkFlex((IntakeConstants.indexerID), MotorType.kBrushless);
  private final SparkFlex spindexer =
      new SparkFlex((IntakeConstants.spindexerID), MotorType.kBrushless);
  private final SparkFlexConfig config = new SparkFlexConfig();

  private final SparkClosedLoopController controller = indexer.getClosedLoopController();

  /** Creates a new Flywheel. */
  public Flywheel(FlywheelIO io) {
    this.io = io;
    io.getHardware().forEach(e -> registerHardware(e));
    registerCheckCommand(systemCheckCommand());
    config.closedLoop.pid(0.0002, 0, 0.005, ClosedLoopSlot.kSlot0);
    config.closedLoop.feedForward.kV(0.00017);
    indexer.configureAsync(
        config, ResetMode.kNoResetSafeParameters, PersistMode.kNoPersistParameters);
  }

  public void runSpindexer() {
    spindexer.set(0.1);
  }

  public void stopSpindexer() {
    spindexer.set(0);
  }

  @Override
  public void periodic() {
    io.updateInputs(inputs);
    Logger.processInputs("Flywheel", inputs);
    isShooting = io.isShooting();
    isReadyToShoot = io.readyToShoot();
  }

  public void setVelocity(double velocityRPM) {
    io.runVelocity(velocityRPM);
  }

  public double getRPM() {
    return inputs.velocityRPM;
  }

  public boolean readyToShoot() {
    return io.readyToShoot();
  }

  public boolean isShooting() {
    return io.isShooting();
  }

  public void runShooter() {
    indexer.set(-1);
    io.runVelocity(1);
    // if (Math.abs(inputs.velocityRPM - indexer.get))
  }

  public void stopShooter() {
    indexer.set(0);
    io.runVelocity(0);
  }

  public boolean noGamePiecesToShoot() {
    return io.noGamePiecesToShoot();
  }

  @Override
  public Command systemCheckCommand() {
    return Commands.sequence(
        Commands.runOnce(() -> io.runVelocity(1)),
        Commands.waitSeconds(0.3),
        Commands.runOnce(
            () -> {
              if (getRPM() < 0) {
                addSubsystemFault(
                    "FLY", "[System Check] Flywheel did not reach desired velocity RPM");
              }
            }));
  }

  public void stopMotors() {
    indexer.getClosedLoopController().setSetpoint(0, ControlType.kVelocity);
    setVelocity(0);
    spindexer.set(0);
  }

  public void testing() {
    controller.setSetpoint(3000, ControlType.kVelocity, ClosedLoopSlot.kSlot0);
    io.runVelocity(10000);
    // spindexer.stopMotor();
    // indexer.stopMotor();
  }

  public void runIndexerAndFlywheel(double velocityRPM) {
    io.runVelocity(velocityRPM);
    controller.setSetpoint(-3000, ControlType.kVelocity, ClosedLoopSlot.kSlot0);
    if (Math.abs(inputs.velocityRPM) > (velocityRPM * 0.9)
        && Math.abs(indexer.getEncoder().getVelocity()) > 10) {
      runSpindexer();
    }
  }
}
