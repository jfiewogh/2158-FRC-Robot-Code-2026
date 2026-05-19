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

<<<<<<<< HEAD:src/main/java/org/austincans/lib/lidar/impl/errors/Timeout.java
package org.austincans.lib.lidar.impl.errors;

import com.google.auto.value.AutoValue;
import org.austincans.lib.lidar.impl.ErrorBase;
========
package org.austincans.lib.sensors.lidar.impl.errors;

import static edu.wpi.first.units.Units.RPM;
import static edu.wpi.first.units.Units.Volts;
>>>>>>>> develop:src/main/java/org/austincans/frc2026/subsystems/feeder/FeederConstants.java

import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.units.measure.Voltage;
import org.austincans.frc2026.Constants;

public class FeederConstants {
  public static final int indexerId = 21;
  public static final int spindexerId = 20;

  public static final AngularVelocity indexerVelocity = RPM.of(-3000.0);
  public static final Voltage spindexerVoltage = Volts.of(0.2 * Constants.maxVoltage);

  public static final double indexerAtSetpointThresholdRPM = 10.0;
}
