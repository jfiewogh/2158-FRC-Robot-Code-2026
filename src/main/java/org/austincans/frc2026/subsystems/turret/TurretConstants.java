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

package org.austincans.frc2026.subsystems.turret;

import com.ctre.phoenix6.CANBus;
import edu.wpi.first.math.util.Units;
import org.austincans.frc2026.util.ConfigUtil;

public class TurretConstants {
  // values used by SparkFlex and TalonFX
  public static final double kP = ConfigUtil.getConfigDouble("TURRET_P", 0);
  public static final double kI = ConfigUtil.getConfigDouble("TURRET_I", 0);
  public static final double kD = ConfigUtil.getConfigDouble("TURRET_D", 0);
  public static final double kV = ConfigUtil.getConfigDouble("TURRET_V", 0);
  public static final double kS = ConfigUtil.getConfigDouble("TURRET_S", 0);
  public static final double kA = ConfigUtil.getConfigDouble("TURRET_A", 0);

  public static final int turretMotorId = ConfigUtil.getConfigInt("TURRET_MOTOR_ID", 0);
  public static final int turretEncoderId = ConfigUtil.getConfigInt("TURRET_ENCODER_ID", 0);

  public static final double gearing = ConfigUtil.getConfigDouble("TURRET_GEARING", 2);
  public static final double turretHeightMeters =
      ConfigUtil.getConfigDouble("TURRET_HEIGHT", Units.inchesToMeters(20));

  public static final double maxVelocityRotationsPerSec = 0.2;
  public static final double maxAccelerationRotationsPerSecSq = 1;

  public static final boolean forwardSoftLimitEnabled =
      ConfigUtil.getConfigBoolean("TURRET_FORWARD_SOFT_LIMIT_ENABLED", true);
  public static final boolean reverseSoftLimitEnabled =
      ConfigUtil.getConfigBoolean("TURRET_REVERSE_SOFT_LIMIT_ENABLED", true);
  public static final double forwardSoftLimit =
      ConfigUtil.getConfigDouble("TURRET_FORWARD_SOFT_LIMIT", 0);
  public static final double reverseSoftLimit =
      ConfigUtil.getConfigDouble("TURRET_REVERSE_SOFT_LIMIT", 0);
  public static final boolean isBrakeMode =
      ConfigUtil.getConfigBoolean("TURRET_IS_BRAKE_MODE", false);

  // values only used by TalonFX
  public static final CANBus turretCANBus = new CANBus(ConfigUtil.getConfigString("*", "2158_CAN"));

  // values only used by SparkFlex
  public static final boolean isBrushlessMotor =
      ConfigUtil.getConfigBoolean("TURRET_IS_BRUSHLESS_MOTOR", true);
  public static final int smartCurrentLimit =
      ConfigUtil.getConfigInt("TURRET_SMART_CURRENT_LIMIT", 80);

  // values (possibly only) used in sim
  public static final double rotationalInertiaMetersSq =
      ConfigUtil.getConfigDouble("TURRET_ROTATIONAL_INERTIA_METER_SQ", 0.01);

  public static final double turretMinimumRotation =
      ConfigUtil.getConfigDouble("TURRET_MIN_ROT", 0);
  public static final double turretMaxRotation = ConfigUtil.getConfigDouble("TURRET_MAX_ROT", 0);
}
