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
//
// Updated on Saturday February 21, 2026 at 12:49:35 CST

package org.austincans.frc2158.subsystems.turret;

import com.ctre.phoenix6.CANBus;
import org.austincans.frc2158.util.ConfigUtil;

public class TurretConstants {
  public static final int turretMotorId = ConfigUtil.getConfigInt("turretID", 0);
  public static final int turretEncoderId = ConfigUtil.getConfigInt("turretEncoderID", 0);

  // TODO: is CANBus name supposed to be intakeCanbus?
  public static final CANBus turretCANBus =
      new CANBus(ConfigUtil.getConfigString("intakeCanbus", "2158"));

  public static final double gearing = 1;

  public static final double turretHeight = 0.762;

  // Turret Sim
  public static final double rotationalInertiaMetersSq = 0.01;
}
