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

import com.ctre.phoenix6.CANBus;
import org.austincans.frc2026.util.ConfigUtil;

public class FlywheelConstants {
  // TODO: get from config
  public static final double kS = ConfigUtil.getConfigDouble("FLYWHEEL_S", 0);
  public static final double kV = ConfigUtil.getConfigDouble("FLYWHEEL_V", 0);
  public static final double kA = ConfigUtil.getConfigDouble("FLYWHEEL_A", 0);
  public static final double flywheelGearRation = ConfigUtil.getConfigDouble("FLYWHEEL_GEAR_RATIO", 2);
  public static final int maxDistFromLasarCan1 =
      ConfigUtil.getConfigInt("FLYWHEEL_LASER_CAN_1", 30);
  public static final int maxDistFromLasarCan2 =
      ConfigUtil.getConfigInt("FLYWHEEL_LASER_CAN_2", 30);

  /* REAL */
  public static final int motor1Id = ConfigUtil.getConfigInt("FLYWHEEL_MOTOR_1_ID", 22);
  public static final int motor2Id = ConfigUtil.getConfigInt("FLYWHEEL_MOTOR_2_ID", 23);
  public static final int encoderId = ConfigUtil.getConfigInt("FLYWHEEL_ENCODER_ID", 0);
  public static final int lasarCan1Id = ConfigUtil.getConfigInt("FLYWHEEL_LASER_CAN_ID_1", 0);
  public static final int lasarCan2Id = ConfigUtil.getConfigInt("FLYWHEEL_LASER_CAN_ID_2", 1);
  public static final CANBus canBus = new CANBus("*");

  public static final double radiusMeters = 0.0508;

  public static final double gearing = ConfigUtil.getConfigDouble("FLYWHEEL_GEARING", 0);

  /* SIM */
  public static final double JKgMetersSq = ConfigUtil.getConfigDouble("FLYWHEEL_KG_METER_SQ", 0);

  public static final double maxRPM = 2100;
}
