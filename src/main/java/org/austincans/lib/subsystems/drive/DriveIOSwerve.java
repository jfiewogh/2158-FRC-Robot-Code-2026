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

package org.austincans.lib.subsystems.drive;

import java.util.List;
import org.austincans.lib.Constants;
import org.austincans.lib.motors.MotorIO;
import org.austincans.lib.subsystems.drive.ModuleIO;
import edu.wpi.first.math.Vector;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.math.numbers.N2;

public abstract class DriveIOSwerve<Motor extends MotorIO, Gyro extends GyroIO> implements DriveIO {
  private final SwerveDriveKinematics kinematics;

  private final ModuleIO[] modules; // FL, FR, BL, FR

  public DriveIOSwerve(DriveConfig driveConfig, ModuleIO frontLeft, ModuleIO frontRight, ModuleIO backLeft, ModuleIO backRight) {
    Translation2d[] moduleTranslations = {
      new Translation2d(driveConfig.wheelbaseMeters() / 2, driveConfig.trackWidthMeters() / 2),
      new Translation2d(driveConfig.wheelbaseMeters() / 2, -driveConfig.trackWidthMeters() / 2),
      new Translation2d(-driveConfig.wheelbaseMeters() / 2, driveConfig.trackWidthMeters() / 2),
      new Translation2d(-driveConfig.wheelbaseMeters() / 2, -driveConfig.trackWidthMeters() / 2)
    };
    kinematics = new SwerveDriveKinematics(moduleTranslations);
    modules = new ModuleIO[] {frontLeft, frontRight, backLeft, backRight};
  }

  private SwerveModulePosition[] getModulePositions() {
    SwerveModulePosition[] states = new SwerveModulePosition[4];
    for (int i = 0; i < 4; i++) {
      states[i] = modules[i].getPosition();
    }
    return states;
  }
  
  public void updateInputs(DriveIOInputs inputs) {
    modules[0].updateInputs(inputs.frontLeftInputs);
    modules[1].updateInputs(inputs.frontRightInputs);
    modules[2].updateInputs(inputs.backLeftInputs);
    modules[3].updateInputs(inputs.backRightInputs);
  }

  /**
   * Runs the drive at the desired velocity.
   * @param speeds Speeds in meters/sec
   */
  @Override
  public void runVelocities(ChassisSpeeds velocities) {
    ChassisSpeeds discreteVelocities = ChassisSpeeds.discretize(velocities, Constants.loopPeriodSec);
    SwerveModuleState[] setpointStatesUnoptimized = kinematics.toSwerveModuleStates(discreteVelocities);
  }

  /**
   * Runs the drive at the desired velocity with setpoint module forces.
   * @param speeds Speeds in meters/sec
   * @param moduleForces The forces applied to each module
   */
  @Override
  public void runVelocities(ChassisSpeeds velocities, List<Vector<N2>> moduleForces) {
  }

  public void runCharacterization() {

  }

  public void runAngularCharacterization() {
  }
  

  @Override
  public void resetPose2d(Pose2d pose) {

  }
}
