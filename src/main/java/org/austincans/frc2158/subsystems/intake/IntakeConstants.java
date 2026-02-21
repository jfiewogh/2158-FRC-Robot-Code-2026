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
// Updated on Thursday February 19, 2026 at 20:08:44 CST

package org.austincans.frc2158.subsystems.intake;

import com.ctre.phoenix6.CANBus;
import org.austincans.frc2158.util.ConfigUtil;

public class IntakeConstants {
  public static final CANBus intakeCANBus =
      new CANBus(ConfigUtil.getConfigString("intakeCanbus", "2158"));

  // TODO: get from config

  public static final double deployGearing = 1;
  public static final double intakeGearing = 1; // placeholder

  public static final double deployRotationalInertiaMetersSq = 0.01;
  public static final double intakeRotationalInertiaMetersSq = 0.01;
  public static final int maxIntakeAmount = ConfigUtil.getConfigInt("max_intake_Amount", 35);
  public static final int maxDistFromIntake = ConfigUtil.getConfigInt("lasarCan_Distance", 20);
}
