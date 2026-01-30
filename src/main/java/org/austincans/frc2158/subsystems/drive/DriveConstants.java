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
// Updated on Thursday January 29, 2026 at 19:31:22 CST

package org.austincans.frc2158.subsystems.drive;

import com.ctre.phoenix6.CANBus;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.util.Units;
import lombok.Builder;
import org.austincans.frc2158.Constants;
import org.austincans.frc2158.Constants.RobotType;
import org.austincans.frc2158.generated.TunerConstants;
import org.austincans.frc2158.util.ConfigUtil;
import org.austincans.frc2158.util.swerve.ModuleLimits;

// TODO Review with CAD Team
public class DriveConstants {
  public static final double odometryFrequency = ConfigUtil.getConfigDouble("ODOMETRY_FREQ", 250);
  public static final double trackWidthX = ConfigUtil.getConfigDouble("ROBOT_TRACK_WIDTH_X", 0);
  public static final double trackWidthY = ConfigUtil.getConfigDouble("ROBOT_TRACK_WIDTH_Y", 0);
  public static final double driveBaseRadius = Math.hypot(trackWidthX / 2, trackWidthY / 2);
  public static final double maxLinearSpeed =
      ConfigUtil.getConfigDouble("ROBOT_MAX_LINEAR_SPEED", 0);
  public static final double maxAngularSpeed = maxLinearSpeed / driveBaseRadius;
  public static final double maxLinearAcceleration =
      ConfigUtil.getConfigDouble("ROBOT_MAX_LINEAR_ACCELERATION", 0);

  /** Includes bumpers! */
  public static final double robotWidth = ConfigUtil.getConfigDouble("ROBOT_WIDTH", 0);

  // Modules
  public static final CANBus moduleCANBus =
      new CANBus(TunerConstants.DrivetrainConstants.CANBusName);

  public static final double driveReduction = (50.0 / 14.0) * (16.0 / 28.0) * (45.0 / 15.0);
  public static final double turnReduction = (150.0 / 7.0);

  public static final double driveCurrentLimitAmps =
      ConfigUtil.getConfigDouble("DRIVE_CURRENT_LIMIT", 0); // placeholder
  public static final double turnCurrentLimitAmps =
      ConfigUtil.getConfigDouble("TURN_CURRENT_LIMIT", 0); // placeholder

  public static final Translation2d[] moduleTranslations = {
    new Translation2d(trackWidthX / 2, trackWidthY / 2),
    new Translation2d(trackWidthX / 2, -trackWidthY / 2),
    new Translation2d(-trackWidthX / 2, trackWidthY / 2),
    new Translation2d(-trackWidthX / 2, -trackWidthY / 2)
  };

  public static final double wheelRadius = Units.inchesToMeters(ConfigUtil.getConfigDouble("c", 0));

  public static final ModuleLimits moduleLimitsFree =
      new ModuleLimits(maxLinearSpeed, maxLinearAcceleration, Units.degreesToRadians(1080.0));

  public static final ModuleConfig[] moduleConfigsComp = {
    // FL
    ModuleConfig.builder()
        .driveMotorId(ConfigUtil.getConfigInt("FL_DRIVE_ID", 0))
        .turnMotorId(ConfigUtil.getConfigInt("FL_TURN_ID", 0))
        .encoderChannel(ConfigUtil.getConfigInt("FL_ENCODER_ID", 0))
        .encoderOffset(Rotation2d.fromRadians(ConfigUtil.getConfigDouble("FL_ENCODER_OFFSET", 0)))
        .turnInverted(ConfigUtil.getConfigBoolean("FL_TURN_INVERTED", true))
        .encoderInverted(ConfigUtil.getConfigBoolean("FL_ENCODER_INVERTED", false))
        .build(),
    // FR
    ModuleConfig.builder()
        .driveMotorId(ConfigUtil.getConfigInt("FR_DRIVE_ID", 0))
        .turnMotorId(ConfigUtil.getConfigInt("FR_TURN_ID", 0))
        .encoderChannel(ConfigUtil.getConfigInt("FR_ENCODER_ID", 0))
        .encoderOffset(Rotation2d.fromRadians(ConfigUtil.getConfigDouble("FR_ENCODER_OFFSET", 0)))
        .turnInverted(ConfigUtil.getConfigBoolean("FR_TURN_INVERTED", true))
        .encoderInverted(ConfigUtil.getConfigBoolean("FR_ENCODER_INVERTED", false))
        .build(),
    // BL
    ModuleConfig.builder()
        .driveMotorId(ConfigUtil.getConfigInt("BL_DRIVE_ID", 0))
        .turnMotorId(ConfigUtil.getConfigInt("BL_TURN_ID", 0))
        .encoderChannel(ConfigUtil.getConfigInt("BL_ENCODER_ID", 0))
        .encoderOffset(Rotation2d.fromRadians(ConfigUtil.getConfigDouble("BL_ENCODER_OFFSET", 0)))
        .turnInverted(ConfigUtil.getConfigBoolean("BL_TURN_INVERTED", true))
        .encoderInverted(ConfigUtil.getConfigBoolean("BL_ENCODER_INVERTED", false))
        .build(),
    // BR
    ModuleConfig.builder()
        .driveMotorId(ConfigUtil.getConfigInt("BR_DRIVE_ID", 0))
        .turnMotorId(ConfigUtil.getConfigInt("BR_TURN_ID", 0))
        .encoderChannel(ConfigUtil.getConfigInt("BR_ENCODER_ID", 0))
        .encoderOffset(Rotation2d.fromRadians(ConfigUtil.getConfigDouble("BR_ENCODER_OFFSET", 0)))
        .turnInverted(ConfigUtil.getConfigBoolean("BR_TURN_INVERTED", true))
        .encoderInverted(ConfigUtil.getConfigBoolean("BR_ENCODER_INVERTED", false))
        .build()
  };

  public static final ModuleConfig[] moduleConfigsDev = {
    // FL
    ModuleConfig.builder()
        .driveMotorId(ConfigUtil.getConfigInt("FL_DRIVE_ID", 0))
        .turnMotorId(ConfigUtil.getConfigInt("FL_TURN_ID", 0))
        .encoderChannel(ConfigUtil.getConfigInt("FL_ENCODER_ID", 0))
        .encoderOffset(Rotation2d.fromRadians(ConfigUtil.getConfigDouble("FL_ENCODER_OFFSET", 0)))
        .turnInverted(ConfigUtil.getConfigBoolean("FL_TURN_INVERTED", true))
        .encoderInverted(ConfigUtil.getConfigBoolean("FL_ENCODER_INVERTED", false))
        .build(),
    // FR
    ModuleConfig.builder()
        .driveMotorId(ConfigUtil.getConfigInt("FR_DRIVE_ID", 0))
        .turnMotorId(ConfigUtil.getConfigInt("FR_TURN_ID", 0))
        .encoderChannel(ConfigUtil.getConfigInt("FR_ENCODER_ID", 0))
        .encoderOffset(Rotation2d.fromRadians(ConfigUtil.getConfigDouble("FR_ENCODER_OFFSET", 0)))
        .turnInverted(ConfigUtil.getConfigBoolean("FR_TURN_INVERTED", true))
        .encoderInverted(ConfigUtil.getConfigBoolean("FR_ENCODER_INVERTED", false))
        .build(),
    // BL
    ModuleConfig.builder()
        .driveMotorId(ConfigUtil.getConfigInt("BL_DRIVE_ID", 0))
        .turnMotorId(ConfigUtil.getConfigInt("BL_TURN_ID", 0))
        .encoderChannel(ConfigUtil.getConfigInt("BL_ENCODER_ID", 0))
        .encoderOffset(Rotation2d.fromRadians(ConfigUtil.getConfigDouble("BL_ENCODER_OFFSET", 0)))
        .turnInverted(ConfigUtil.getConfigBoolean("BL_TURN_INVERTED", true))
        .encoderInverted(ConfigUtil.getConfigBoolean("BL_ENCODER_INVERTED", false))
        .build(),
    // BR
    ModuleConfig.builder()
        .driveMotorId(ConfigUtil.getConfigInt("BR_DRIVE_ID", 0))
        .turnMotorId(ConfigUtil.getConfigInt("BR_TURN_ID", 0))
        .encoderChannel(ConfigUtil.getConfigInt("BR_ENCODER_ID", 0))
        .encoderOffset(Rotation2d.fromRadians(ConfigUtil.getConfigDouble("BR_ENCODER_OFFSET", 0)))
        .turnInverted(ConfigUtil.getConfigBoolean("BR_TURN_INVERTED", true))
        .encoderInverted(ConfigUtil.getConfigBoolean("BR_ENCODER_INVERTED", false))
        .build()
  };
  public static final double robotMassKG = ConfigUtil.getConfigDouble("ROBOT_MASS", 0);
  public static final double robotMOI = ConfigUtil.getConfigDouble("ROBOT_MOI", 0);
  ;
  public static final double wheelCOF = ConfigUtil.getConfigDouble("ROBOT_COF", 0);
  ;
  public static final double driveP = ConfigUtil.getConfigDouble("DRIVE_P", 0);
  public static final double driveD = ConfigUtil.getConfigDouble("DRIVE_D", 0);
  public static final double turnP = ConfigUtil.getConfigDouble("TURN_P", 0);
  public static final double turnD = ConfigUtil.getConfigDouble("TURN_D", 0);
  public static final double speedConstant = ConfigUtil.getConfigDouble("SPEED_CONSTANT", 0);

  public static class PigeonConstants {
    public static final int id = Constants.getRobot() == RobotType.DEVBOT ? 3 : 30;
  }

  @Builder
  public record ModuleConfig(
      int driveMotorId,
      int turnMotorId,
      int encoderChannel,
      Rotation2d encoderOffset,
      boolean turnInverted,
      boolean encoderInverted) {}
}
