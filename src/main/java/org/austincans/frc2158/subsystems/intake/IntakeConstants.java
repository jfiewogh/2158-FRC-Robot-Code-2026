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

package org.austincans.frc2158.subsystems.intake;

import com.ctre.phoenix6.CANBus;
import edu.wpi.first.math.controller.ArmFeedforward;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import org.austincans.frc2158.util.ConfigUtil;

public class IntakeConstants {
  public static final CANBus intakeCANBus =
      new CANBus(ConfigUtil.getConfigString("INTAKE_CANBUS", "2158"));

  public static final int deployCanID = ConfigUtil.getConfigInt("INTAKE_DEPLOY_ID", 0);
  public static final int intakeCanID = ConfigUtil.getConfigInt("INTAKE_MOTOR_ID", 0);

  /*
   * PID
   * kP: Proportional
   * kI: Integral
   * kD: Derivative
   */
  public static double kP = ConfigUtil.getConfigDouble("INTAKE_P", 0.001);
  public static double kI = ConfigUtil.getConfigDouble("INTAKE_I", 0);
  public static double kD = ConfigUtil.getConfigDouble("INTAKE_D", 0);

  public static double maxVelocityRadPerSec = Math.PI;
  public static TrapezoidProfile.Constraints constraints =
      new TrapezoidProfile.Constraints(
          ConfigUtil.getConfigDouble("INTAKE_MAX_VEL_RAD", maxVelocityRadPerSec),
          ConfigUtil.getConfigDouble("INTAKE_MAX_ACCEL_RAD_2", Math.PI));

  /** Feedforward kS: Static kG: Gravity kV: Velocity kA: Acceleration */
  public static double kS = ConfigUtil.getConfigDouble("INTAKE_S", 0);

  public static double kG = ConfigUtil.getConfigDouble("INTAKE_G", 0);
  public static double kV = ConfigUtil.getConfigDouble("INTAKE_V", 0);
  public static double kA = ConfigUtil.getConfigDouble("INTAKE_A", 0);
  public static final ArmFeedforward feedforward = new ArmFeedforward(kS, kG, kV, kA);
  public static ProfiledPIDController intakePIDController =
      new ProfiledPIDController(kP, kI, kD, constraints);

  public static boolean deployBreakEnabled = ConfigUtil.getConfigBoolean("INTAKE_BRAKE_MODE", true);
  public static boolean brakeMode = ConfigUtil.getConfigBoolean("INTAKE_BRAKE_MODE", false);
  public static final double intakeGearing = ConfigUtil.getConfigDouble("INTAKE_GEAR_RATIO", 1);
  public static final double deployGearing =
      ConfigUtil.getConfigDouble("INTAKE_DEPLOY_GEAR_RATIO", 1);

  public static final double deployRotationalInertiaMetersSq =
      ConfigUtil.getConfigDouble("INTAKE_DEPLOY_ROTATIONAL_INERTIA_METER", 1);
  public static final double intakeRotationalInertiaMetersSq =
      ConfigUtil.getConfigDouble("INTAKE_ROTATIONAL_INERTIA_METERS", 1);
  public static final int maxIntakeAmount = ConfigUtil.getConfigInt("INTAKE_MAX_CAPACITY", 35);
  public static final int maxDistFromIntake =
      ConfigUtil.getConfigInt("INTAKE_LASER_CAN_DISTANCE", 20);

  public static int laserCANID = ConfigUtil.getConfigInt("INTAKE_LASER_CAN_ID", 0);
}
