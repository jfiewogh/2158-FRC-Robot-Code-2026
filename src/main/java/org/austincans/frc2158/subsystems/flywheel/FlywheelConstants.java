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
// Updated on Thursday January 22, 2026 at 19:06:38 CST

package org.austincans.frc2158.subsystems.flywheel;

import org.austincans.frc2158.util.ConfigUtil;

import com.ctre.phoenix6.CANBus;

public class FlywheelConstants {
  // TODO: get from config
  public static final double kS = ConfigUtil.getConfigDouble("FLYWHEEL_S", 0);  
  public static final double kV = ConfigUtil.getConfigDouble("FLYWHEEL_V", 0);
  public static final double kA = ConfigUtil.getConfigDouble("FLYWHEEL_A", 0);

  /* REAL */
  public static final int motorId = ConfigUtil.getConfigInt("FLYWHEEL_MOTOR_ID", 0);
  public static final int encoderId = ConfigUtil.getConfigInt("FLYWHEEL_ENCODER_ID", 0);

  public static final CANBus canBus = new CANBus("*");

  public static final double radiusMeters = ConfigUtil.getConfigDouble("FLYWHEEL_RADIUS", encoderId);

  public static final double gearing = ConfigUtil.getConfigDouble("FLYWHEEL_GEARING", 0);

  /* SIM */
  public static final double JKgMetersSq = ConfigUtil.getConfigDouble("FLYWHEEL_KG_METER_SQ", 0);

  public static final double maxRPM = ConfigUtil.getConfigDouble("FLYWHEEL_MAX_RPM", JKgMetersSq);
}
