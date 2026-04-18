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

<<<<<<<< HEAD:src/main/java/org/austincans/lib/util/DummyLogReceiver.java
package org.austincans.lib.util;
========
package org.austincans.frc2026.subsystems.feeder;
>>>>>>>> develop:src/main/java/org/austincans/frc2026/subsystems/feeder/SpindexerIOSparkFlex.java

import static edu.wpi.first.units.Units.Volts;

import com.revrobotics.spark.SparkFlex;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import org.austincans.frc2026.subsystems.selfcheck.AdvancedSubsystem;

public class SpindexerIOSparkFlex implements SpindexerIO {
  private final SparkFlex motor = new SparkFlex(FeederConstants.spindexerId, MotorType.kBrushless);

  public SpindexerIOSparkFlex() {}

  @Override
  public void updateInputs(SpindexerIOInputs inputs) {
    inputs.voltage = Volts.of(motor.getBusVoltage());
  }

  @Override
  public void applyOutputs(SpindexerIOOutputs outputs) {
    motor.setVoltage(outputs.voltage.in(Volts));
  }

  @Override
  public void registerSelfCheckHardware(AdvancedSubsystem subsystem) {
    subsystem.registerHardware("Spindexer", motor);
  }
}
