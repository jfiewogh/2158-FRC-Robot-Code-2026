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

package org.austincans.frc2158.subsystems.hood;
import java.util.TreeMap;
import org.austincans.frc2158.util.ConfigUtil;
import edu.wpi.first.math.geometry.Rotation2d;

public class HoodConstants {
  static int flexMotorID = ConfigUtil.getConfigInt("HOOD_SPARK_FLEX_MOTOR_ID", 0);
  static int talonMotorID = ConfigUtil.getConfigInt("HOOD_TALON_ID", 0);
  static int linearActuatorID = ConfigUtil.getConfigInt("HOOD_LINEAR_ACTUATOR_ID", 0);
  static double kP = ConfigUtil.getConfigDouble("HOOD_P", 0);
  static double kI = ConfigUtil.getConfigDouble("HOOD_I", 0);
  static double kD = ConfigUtil.getConfigDouble("HOOD_D", 0);
  static double kV = ConfigUtil.getConfigDouble("HOOD_V", 0);
}
