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

package org.austincans.frc2026.subsystems.feeder;

import static edu.wpi.first.units.Units.RPM;

import com.revrobotics.PersistMode;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.ResetMode;
import com.revrobotics.spark.ClosedLoopSlot;
import com.revrobotics.spark.SparkBase.ControlType;
import com.revrobotics.spark.SparkClosedLoopController;
import com.revrobotics.spark.SparkFlex;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.config.SparkFlexConfig;
import org.austincans.frc2026.subsystems.selfcheck.AdvancedSubsystem;

public class IndexerIOSparkFlex implements IndexerIO {
  private final SparkFlex motor;
  private final SparkFlexConfig config;
  private final RelativeEncoder encoder;
  private final SparkClosedLoopController controller;

  public IndexerIOSparkFlex() {
    motor = new SparkFlex(FeederConstants.indexerId, MotorType.kBrushless);

    config = new SparkFlexConfig();
    config.closedLoop.pid(0.0002, 0, 0.005, ClosedLoopSlot.kSlot0);
    config.closedLoop.feedForward.kV(0.00017);
    motor.configureAsync(
        config, ResetMode.kNoResetSafeParameters, PersistMode.kNoPersistParameters);

    encoder = motor.getEncoder();
    controller = motor.getClosedLoopController();
  }

  @Override
  public void updateInputs(IndexerIOInputs inputs) {
    inputs.velocity = RPM.of(encoder.getVelocity());
  }

  @Override
  public void applyOutputs(IndexerIOOutputs outputs) {
    controller.setSetpoint(outputs.velocity.in(RPM), ControlType.kVelocity, ClosedLoopSlot.kSlot0);
  }

  @Override
  public void registerSelfCheckHardware(AdvancedSubsystem subsystem) {
    subsystem.registerHardware("Indexer", motor);
  }
}
