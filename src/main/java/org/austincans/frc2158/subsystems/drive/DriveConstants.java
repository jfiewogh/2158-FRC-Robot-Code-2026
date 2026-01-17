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
// Updated on Saturday January 17, 2026 at 15:50:10 CST

package org.austincans.frc2158.subsystems.drive;

import com.ctre.phoenix6.CANBus;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.util.Units;
import lombok.Builder;
import org.austincans.frc2158.Constants;
import org.austincans.frc2158.Constants.RobotType;
import org.austincans.frc2158.generated.TunerConstants;
import org.austincans.frc2158.util.swerve.ModuleLimits;

// TODO Review with CAD Team
public class DriveConstants {
  public static final double odometryFrequency = 250;
  public static final double trackWidthX =
      Constants.getRobot() == RobotType.DEVBOT
          ? Units.inchesToMeters(20.75)
          : Units.inchesToMeters(22.75);
  public static final double trackWidthY =
      Constants.getRobot() == RobotType.DEVBOT
          ? Units.inchesToMeters(20.75)
          : Units.inchesToMeters(22.75);
  public static final double driveBaseRadius = Math.hypot(trackWidthX / 2, trackWidthY / 2);
  public static final double maxLinearSpeed = 4.69;
  public static final double maxAngularSpeed = 4.69 / driveBaseRadius;
  public static final double maxLinearAcceleration = 22.0;

  /** Includes bumpers! */
  public static final double robotWidth =
      Units.inchesToMeters(28.0) + 2 * Units.inchesToMeters(2.0);

  // Modules
  public static final CANBus moduleCANBus =
      new CANBus(TunerConstants.DrivetrainConstants.CANBusName);

  public static final double driveReduction = (50.0 / 14.0) * (16.0 / 28.0) * (45.0 / 15.0);
  public static final double turnReduction = (150.0 / 7.0);

  public static final double driveCurrentLimitAmps = 40; // placeholder
  public static final double turnCurrentLimitAmps = 40; // placeholder

  public static final Translation2d[] moduleTranslations = {
    new Translation2d(trackWidthX / 2, trackWidthY / 2),
    new Translation2d(trackWidthX / 2, -trackWidthY / 2),
    new Translation2d(-trackWidthX / 2, trackWidthY / 2),
    new Translation2d(-trackWidthX / 2, -trackWidthY / 2)
  };

  public static final double wheelRadius = Units.inchesToMeters(1.9413001940413326);

  public static final ModuleLimits moduleLimitsFree =
      new ModuleLimits(maxLinearSpeed, maxLinearAcceleration, Units.degreesToRadians(1080.0));

  public static final ModuleConfig[] moduleConfigsComp = {
    // FL
    ModuleConfig.builder()
        .driveMotorId(16)
        .turnMotorId(15)
        .encoderChannel(41)
        .encoderOffset(Rotation2d.fromRadians(2.5356702423749646))
        .turnInverted(true)
        .encoderInverted(false)
        .build(),
    // FR
    ModuleConfig.builder()
        .driveMotorId(10)
        .turnMotorId(11)
        .encoderChannel(42)
        .encoderOffset(Rotation2d.fromRadians(-2.932971266437346))
        .turnInverted(true)
        .encoderInverted(false)
        .build(),
    // BL
    ModuleConfig.builder()
        .driveMotorId(18)
        .turnMotorId(19)
        .encoderChannel(43)
        .encoderOffset(Rotation2d.fromRadians(0.6458059116998549))
        .turnInverted(true)
        .encoderInverted(false)
        .build(),
    // BR
    ModuleConfig.builder()
        .driveMotorId(13)
        .turnMotorId(14)
        .encoderChannel(44)
        .encoderOffset(Rotation2d.fromRadians(-2.5187964537082226))
        .turnInverted(true)
        .encoderInverted(false)
        .build()
  };

  public static final ModuleConfig[] moduleConfigsDev = {
    // FL
    ModuleConfig.builder()
        .driveMotorId(12)
        .turnMotorId(9)
        .encoderChannel(1)
        .encoderOffset(Rotation2d.fromRadians(-0.009115335014721037))
        .turnInverted(true)
        .encoderInverted(false)
        .build(),
    // FR
    ModuleConfig.builder()
        .driveMotorId(2)
        .turnMotorId(10)
        .encoderChannel(3)
        .encoderOffset(Rotation2d.fromRadians(0.8427416931125384))
        .turnInverted(true)
        .encoderInverted(false)
        .build(),
    // BL
    ModuleConfig.builder()
        .driveMotorId(15)
        .turnMotorId(11)
        .encoderChannel(0)
        .encoderOffset(Rotation2d.fromRadians(-1.0620197413817225))
        .turnInverted(true)
        .encoderInverted(false)
        .build(),
    // BR
    ModuleConfig.builder()
        .driveMotorId(3)
        .turnMotorId(8)
        .encoderChannel(2)
        .encoderOffset(Rotation2d.fromRadians(-2.600063124240756))
        .turnInverted(true)
        .encoderInverted(false)
        .build()
  };

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
