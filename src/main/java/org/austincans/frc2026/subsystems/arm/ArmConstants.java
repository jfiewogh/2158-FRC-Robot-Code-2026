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

package org.austincans.frc2026.subsystems.arm;

import com.ctre.phoenix6.CANBus;
import edu.wpi.first.math.controller.ArmFeedforward;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import org.austincans.frc2026.util.ConfigUtil;

public class ArmConstants {

  public static double gearRatio = ConfigUtil.getConfigDouble("ARM_GEAR_RATIO", 0);
  public static double armLength = ConfigUtil.getConfigDouble("ARM_LENGTH", 0);
  public static double maxRad = ConfigUtil.getConfigDouble("ARM_MAX_RAD", 0);
  public static double startingPosition = ConfigUtil.getConfigDouble("ARM_STARTING_POSITION", 0);

  /*
   * PID
   * kP: Proportional
   * kI: Integral
   * kD: Derivative
   */
  public static double kP = ConfigUtil.getConfigDouble("ARM_P", 0);
  public static double kI = ConfigUtil.getConfigDouble("ARM_I", 0);
  public static double kD = ConfigUtil.getConfigDouble("ARM_D", 0);

  public static TrapezoidProfile.Constraints constraints =
      new TrapezoidProfile.Constraints(
          ConfigUtil.getConfigDouble("ARM_MAX_VELOCITY_ROTATIONS", 0),
          ConfigUtil.getConfigDouble("ARM_MAX_ACCELERATION", 0));

  /** Feedforward kS: Static kG: Gravity kV: Velocity kA: Acceleration */
  public static double kS = ConfigUtil.getConfigDouble("ARM_S", 0);

  public static double kG = ConfigUtil.getConfigDouble("ARM_G", 0);
  public static double kV = ConfigUtil.getConfigDouble("ARM_V", 0);
  public static double kA = ConfigUtil.getConfigDouble("ARM_A", 0);
  public static final ArmFeedforward feedforward = new ArmFeedforward(kS, kG, kV, kA);
  public static ProfiledPIDController armPIDController =
      new ProfiledPIDController(kP, kI, kD, constraints);

  public static final CANBus armCANBus = new CANBus(ConfigUtil.getConfigString("ARM_CANBUS", ""));
  public static final int canID = ConfigUtil.getConfigInt("ARM_CAN_ID", 0);
  public static final int statorCurrentLimit = ConfigUtil.getConfigInt("ARM_CURRENT_LIMIT", 0);
  public static final boolean brakeMode = ConfigUtil.getConfigBoolean("ARM_BRAKE_MODE", false);
  public static int armEncoder = ConfigUtil.getConfigInt("ARM_ENCODER_ID", 0);
  public static double armForwardSoftLimit =
      ConfigUtil.getConfigDouble("ARM_FORWARD_SOFT_LIMIT", 0);
  public static double armReversedSoftLimit =
      ConfigUtil.getConfigDouble("ARM_REVERSED_SOFT_LIMIT", 0);
  public static double armForwardHardLimit =
      ConfigUtil.getConfigDouble("ARM_FORWARD_HARD_LIMIT", 0);
  public static double armReversedHardLimit =
      ConfigUtil.getConfigDouble("ARM_REVERSED_HARD_LIMIT", 0);
  public static boolean armForwardSoftLimitEnabled =
      ConfigUtil.getConfigBoolean("ARM_FORWARD_SOFT_LIMIT_ENABLED", false);
  public static boolean armForwardHardLimitEnabled =
      ConfigUtil.getConfigBoolean("ARM_FORWARD_HARD_LIMIT_ENABLED", false);
  public static boolean armReversedSoftLimitEnabled =
      ConfigUtil.getConfigBoolean("ARM_REVERSED_SOFT_LIMIT_ENABLED", false);
  public static boolean armReversedHardLimitEnabled =
      ConfigUtil.getConfigBoolean("ARM_REVERSED_HARD_LIMIT_ENABLED", false);
}
