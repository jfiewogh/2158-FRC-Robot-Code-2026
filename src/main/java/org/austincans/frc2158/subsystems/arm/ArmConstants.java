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
// Updated on Tuesday January 20, 2026 at 17:54:53 CST

package org.austincans.frc2158.subsystems.arm;

import org.austincans.frc2158.util.ConfigUtil;

import com.ctre.phoenix6.CANBus;

public class ArmConstants {
  public static double armLength = ConfigUtil.getConfigDouble("ARM_LENGTH", 0);
  public static double armMaxRad = ConfigUtil.getConfigDouble("ARM_MAX_RAD", 0); // Max Forward Motor Movement in Radians
  public static double armStartingPosition = ConfigUtil.getConfigDouble("ARM_STARTING_POSITION", 0);
  public static double armForwardSoftLimit = ConfigUtil.getConfigDouble("ARM_FORWARD_SOFT_LIMIT", 0);
  public static boolean armSoftLimitEnabled = ConfigUtil.getConfigBoolean("ARM_FORWARD_SOFT_LIMIT_ENABLED", false);
  public static double armReverseSoftLimit = ConfigUtil.getConfigDouble("ARM_REVERSED_SOFT_LIMIT", 0);
  public static boolean armReverseSoftLimitEnabled = ConfigUtil.getConfigBoolean("ARM_REVERSED_SOFT_LIMIT_ENALBED", false);
  public static double defaultArmVel = ConfigUtil.getConfigDouble("ARM_DEFAULT_VELOCITY", 0);
  public static int armCurrentLimit = ConfigUtil.getConfigInt("ARM_CURRENT_LIMIT", 0);
  public static boolean armIsBrakeMode = ConfigUtil.getConfigBoolean("ARM_BRAKE_MODE", false);
  public static int armCANID = ConfigUtil.getConfigInt("ARM_CAN_ID", 0);
  public static CANBus armCANBus = new CANBus(ConfigUtil.getConfigString("ARM_CANBUS", ""));
  public static double armGearRatio = ConfigUtil.getConfigDouble("ARM_GEAR_RATIO", 1);

  public static double kP = ConfigUtil.getConfigDouble("ARM_P", 0);
  public static double kI = ConfigUtil.getConfigDouble("ARM_I", 0);
  public static double kD = ConfigUtil.getConfigDouble("ARM_D", 0);
  public static double kS = ConfigUtil.getConfigDouble("ARM_S", 0);
  public static double kG = ConfigUtil.getConfigDouble("ARM_G", 0);
  public static double kV = ConfigUtil.getConfigDouble("ARM_V", 0);
  public static double kA = ConfigUtil.getConfigDouble("ARM_A", 0);

  public static double armMaxAcceleration = ConfigUtil.getConfigDouble("ARM_MAX_ACCELERATION", 0);
  public static double armMaxVelocityRotations = ConfigUtil.getConfigDouble("ARM_MAX_VELOCITY_ROTATIONS", 0);
  public static int armEncoderID = ConfigUtil.getConfigInt("ARM_ENCODER_ID", 0);
}
