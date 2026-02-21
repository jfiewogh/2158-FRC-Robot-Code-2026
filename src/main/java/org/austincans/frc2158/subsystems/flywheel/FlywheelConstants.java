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

package org.austincans.frc2158.subsystems.flywheel;

import com.ctre.phoenix6.CANBus;

public class FlywheelConstants {
  // TODO: get from config
  public static final double kS = ConfigUtil.getConfigDouble("FLYWHEEL_S", 0);
  public static final double kV = ConfigUtil.getConfigDouble("FLYWHEEL_V", 0);
  public static final double kA = ConfigUtil.getConfigDouble("FLYWHEEL_A", 0);
  public static final int maxDistFromLasarCan1 = ConfigUtil.getConfigInt("lasarCan1_distance", 30);
  public static final int maxDistFromLasarCan2 = ConfigUtil.getConfigInt("lasarCan2_distance", 30);

  /* REAL */
  public static final int motorId = ConfigUtil.getConfigInt("FLYWHEEL_MOTOR_ID", 0);
  public static final int encoderId = ConfigUtil.getConfigInt("FLYWHEEL_ENCODER_ID", 0);
  public static final int lasarCan1Id = ConfigUtil.getConfigInt("LASARCAN1_ID", 0);
  public static final int lasarCan2Id = ConfigUtil.getConfigInt("LASARCAN2_ID", 1);
  public static final CANBus canBus = new CANBus("*");

  public static final double radiusMeters = 0.0508;

  public static final double gearing = 1.0;

  /* SIM */
  public static final double JKgMetersSq = 0.001;

  public static final double maxRPM = 2000;
}
