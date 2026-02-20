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

package org.austincans.frc2158.subsystems.drive;

import edu.wpi.first.math.Vector;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.numbers.N2;
import java.util.List;
import org.austincans.frc2158.util.swerve.SwerveSetpoint;
import org.austincans.frc2158.util.swerve.SwerveSetpointGenerator;
import org.littletonrobotics.junction.AutoLog;

public interface DriveIO {
  @AutoLog
  public class DriveIOInputs {
    Pose3d pose = Pose3d.kZero;
    ChassisSpeeds chassisSpeeds = new ChassisSpeeds();
  }

  public static final SwerveDriveKinematics kinematics =
      new SwerveDriveKinematics(DriveConstants.moduleTranslations);

  public static final SwerveSetpointGenerator swerveSetpointGenerator =
      new SwerveSetpointGenerator(kinematics, DriveConstants.moduleTranslations);

  public default void updateInputs(DriveIOInputs inputs) {}

  public default void runVelocity(ChassisSpeeds speeds) {}

  public default void runVelocity(ChassisSpeeds speeds, List<Vector<N2>> moduleForces) {}

  public default void setModules(Module[] modules) {}

  public default void updateSetpoint(SwerveSetpoint setpoint) {}

  public default void resetPose(Pose2d pose) {}

  public default void resetHeadings(Rotation2d[] headings) {
    kinematics.resetHeadings(headings);
  }
}
