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
// Updated on Thursday February 05, 2026 at 19:51:21 CST

package org.austincans.frc2158.subsystems.turret;

import com.ctre.phoenix6.CANBus;
import org.austincans.frc2158.util.ConfigUtil;

public class TurretConstants {
  public static final double kP = ConfigUtil.getConfigDouble("TURRET_P", 0);
  public static final double kI = ConfigUtil.getConfigDouble("TURRET_I", 0);
  public static final double kD = ConfigUtil.getConfigDouble("TURRET_D", 0);
  public static final double kV = ConfigUtil.getConfigDouble("TURRET_V", 0);
  public static final double kS = ConfigUtil.getConfigDouble("TURRET_S", 0);
  public static final double kA = ConfigUtil.getConfigDouble("TURRET_A", 0);

  public static final int turretMotorId = ConfigUtil.getConfigInt("TURRET_MOTOR_ID", 0);
  public static final int turretEncoderId = ConfigUtil.getConfigInt("TURRET_ENCODER_ID", 0);

  // TODO: is CANBus name supposed to be intakeCanbus?
  public static final CANBus turretCANBus = new CANBus(ConfigUtil.getConfigString("*", "2158"));

  public static final double gearing = ConfigUtil.getConfigDouble("TURRET_GEARING", 0);

  public static final double turretHeight = ConfigUtil.getConfigDouble("TURRET_HEIGHT", 0);

  public static final boolean forwardLimit = true;
  public static final boolean reverseLimit = true;
  public static final double turretForwardSoftLimit =
      ConfigUtil.getConfigDouble("TURRET_FORWARD_SOFT_LIMIT", 0);
  public static final double turretReverseSoftLimit =
      ConfigUtil.getConfigDouble("TURRET_FORWARD_SOFT_LIMIT", 0);

  // Turret Sim
  public static final double rotationalInertiaMetersSq =
      ConfigUtil.getConfigDouble("TURRET_ROTATIONAL_INERTIA_METER_SQ", 0);
}
