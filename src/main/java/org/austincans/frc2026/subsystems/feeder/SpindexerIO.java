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

<<<<<<<< HEAD:src/main/java/org/austincans/lib/lidar/impl/vl53l0x/errors/NoSpadInfo.java
package org.austincans.lib.lidar.impl.vl53l0x.errors;

import com.google.auto.value.AutoValue;
import org.austincans.lib.lidar.impl.ErrorBase;
========
package org.austincans.frc2026.subsystems.feeder;

import static edu.wpi.first.units.Units.Volts;
>>>>>>>> develop:src/main/java/org/austincans/frc2026/subsystems/feeder/SpindexerIO.java

import edu.wpi.first.units.measure.Voltage;
import org.austincans.frc2026.subsystems.selfcheck.AdvancedSubsystem;
import org.littletonrobotics.junction.AutoLog;

public interface SpindexerIO {
  @AutoLog
  public class SpindexerIOInputs {
    public Voltage voltage = Volts.of(0);
  }

  public class SpindexerIOOutputs {
    public Voltage voltage = Volts.of(0);
  }

  public void updateInputs(SpindexerIOInputs inputs);

  public void applyOutputs(SpindexerIOOutputs outputs);

  public void registerSelfCheckHardware(AdvancedSubsystem subsystem);
}
