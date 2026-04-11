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

import static edu.wpi.first.units.Units.Amps;
import static edu.wpi.first.units.Units.RPM;
import static edu.wpi.first.units.Units.RotationsPerSecond;

import com.revrobotics.PersistMode;
import com.revrobotics.ResetMode;
import com.revrobotics.spark.ClosedLoopSlot;
import com.revrobotics.spark.SparkBase.ControlType;
import com.revrobotics.spark.SparkClosedLoopController;
import com.revrobotics.spark.SparkFlex;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.config.SparkFlexConfig;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import lombok.Getter;
import lombok.experimental.Accessors;
import org.austincans.frc2026.Constants;
import org.austincans.frc2026.Robot;
import org.austincans.frc2026.subsystems.feeder.FeederConstants;
import org.austincans.frc2026.subsystems.flywheel.FlywheelIO.FlywheelIOOutputMode;
import org.austincans.frc2026.subsystems.flywheel.FlywheelIO.FlywheelIOOutputs;
import org.austincans.frc2026.subsystems.selfcheck.AdvancedSubsystem;
import org.austincans.frc2026.util.EqualsUtil;
import org.littletonrobotics.junction.AutoLogOutput;
import org.littletonrobotics.junction.Logger;

/** Flywheel, Indexer, and Spindexer Subsystem */
// TODO: maybe split into Flywheel and Indexer subsystems
public class Flywheel extends AdvancedSubsystem {
  private final FlywheelIO io;
  private final FlywheelIOInputsAutoLogged inputs = new FlywheelIOInputsAutoLogged();
  private final FlywheelIOOutputs outputs = new FlywheelIOOutputs();

  private final SparkFlex indexer = new SparkFlex(FeederConstants.indexerId, MotorType.kBrushless);
  private final SparkFlexConfig indexerConfig = new SparkFlexConfig();
  private final SparkClosedLoopController indexerController = indexer.getClosedLoopController();

  private final SparkFlex spindexer =
      new SparkFlex(FeederConstants.spindexerId, MotorType.kBrushless);

  @Getter
  @Accessors(fluent = true)
  @AutoLogOutput(key = "Flywheel/AtGoal")
  private boolean atGoal = false;

  /** Creates a new Flywheel. */
  public Flywheel(FlywheelIO io) {
    this.io = io;
    io.registerSelfCheckHardware(this);
    registerCheckCommand(systemCheckCommand());

    indexerConfig.closedLoop.pid(0.0002, 0, 0.005, ClosedLoopSlot.kSlot0);
    indexerConfig.closedLoop.feedForward.kV(0.00017);
    indexer.configureAsync(
        indexerConfig, ResetMode.kNoResetSafeParameters, PersistMode.kNoPersistParameters);
  }

  @Override
  public void periodic() {
    io.updateInputs(inputs);
    Logger.processInputs("Flywheel", inputs);

    double velocityRadPerSec = inputs.velocity.in(RPM);
    double setpointRadPerSec = outputs.velocity.in(RPM);
    atGoal = EqualsUtil.epsilonEquals(setpointRadPerSec, velocityRadPerSec, 2.0);
    Logger.recordOutput("Flywheel/Setpoint", setpointRadPerSec);
    Logger.recordOutput("Flywheel/Goal", velocityRadPerSec);

    Robot.batteryLogger.reportCurrentUsage(
        "Flywheel", false, inputs.current.in(Amps), inputs.current.in(Amps));
  }

  @Override
  public void periodicAfterScheduler() {
    io.applyOutputs(outputs);
    // LoggedTracer.record("Flywheel/AfterScheduler");
  }

  /** Run closed loop at the specified velocity. */
  public void setVelocity(AngularVelocity velocity) {
    outputs.mode = FlywheelIOOutputMode.VELOCITY;
    outputs.velocity = velocity;
  }

  public void stop() {
    setVelocity(RPM.of(0));
  }

  public void stopMotors() {
    indexerController.setSetpoint(0, ControlType.kVelocity);
    setVelocity(RotationsPerSecond.of(0));
    spindexer.set(0);
  }

  public void runIndexerAndFlywheel(double velocityRPM) {
    setVelocity(RPM.of(velocityRPM));
    indexerController.setSetpoint(-3000, ControlType.kVelocity, ClosedLoopSlot.kSlot0);
    if (Math.abs(inputs.velocity.in(RPM)) > (velocityRPM * 0.9)
        && Math.abs(indexer.getEncoder().getVelocity()) > 10) {
      runSpindexer();
    } else {
      stopSpindexer();
    }
  }

  public void runSpindexer() {
    spindexer.setVoltage(0.2 * Constants.maxVoltage);
  }

  public void stopSpindexer() {
    spindexer.setVoltage(0);
  }

  public double getRPM() {
    return inputs.velocity.in(RPM);
  }

  public boolean atSetpoint() {
    return outputs.mode == FlywheelIOOutputMode.VELOCITY
        && Math.abs(getRPM() - outputs.velocity.in(RPM)) < FlywheelConstants.atSetpointThresholdRPM;
  }

  @Override
  public Command systemCheckCommand() {
    return Commands.sequence(
        Commands.run(() -> setVelocity(RPM.of(100)), this).withTimeout(5),
        Commands.waitSeconds(2),
        Commands.runOnce(
            () -> {
              if (getRPM() < 0) {
                addSubsystemFault(
                    "FLY", "[System Check] Flywheel did not reach desired velocity RPM");
              }
            },
            this),
        Commands.runOnce(() -> setVelocity(RPM.of(0)), this));
  }
}
