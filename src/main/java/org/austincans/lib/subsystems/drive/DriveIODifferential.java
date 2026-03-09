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

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.DifferentialDriveKinematics;
import edu.wpi.first.math.kinematics.DifferentialDriveOdometry;
import edu.wpi.first.math.kinematics.DifferentialDriveWheelSpeeds;
import org.austincans.lib.configs.DriveConfigDifferential;
import org.austincans.lib.motors.MotorIO;

// TODO: Finish this class
public class DriveIODifferential implements DriveIO {
  private final DifferentialDriveKinematics kinematics;
  private final MotorIO leftLeader;
  private final MotorIO leftFollower;
  private final MotorIO rightLeader;
  private final MotorIO rightFollower;
  private final GyroIO gyroIO;

  private final double wheelRadiusMeters;
  private final DifferentialDriveOdometry odometry;

  public DriveIODifferential(DriveConfigDifferential driveConfig, MotorIO leftLeader, MotorIO leftFollower, MotorIO rightLeader, MotorIO rightFollower, GyroIO gyroIO) {
    kinematics = new DifferentialDriveKinematics(driveConfig.trackWidthMeters);
    this.leftLeader = leftLeader;
    this.leftFollower = leftFollower;
    this.rightLeader = rightLeader;
    this.rightFollower = rightFollower;
    this.gyroIO = gyroIO;
    wheelRadiusMeters = driveConfig.wheelRadiusMeters;
    odometry = new DifferentialDriveOdometry(null, null, null);

  }

  @Override
  public void updateInputs(DriveIOInputs inputs) {
    inputs.pose = getPose();
    inputs.velocities = getRobotRelativeVelocities();
  }

  @Override
  public void runVelocities(ChassisSpeeds velocities) {
    DifferentialDriveWheelSpeeds wheelVelocities = kinematics.toWheelSpeeds(velocities);
    double leftVelocity = wheelVelocities.leftMetersPerSecond / wheelRadiusMeters;
    double rightVelocity = wheelVelocities.rightMetersPerSecond / wheelRadiusMeters;
    leftLeader.setVelocity(leftVelocity);
    leftFollower.setVelocity(leftVelocity);
    rightLeader.setVelocity(rightVelocity);
    rightFollower.setVelocity(rightVelocity);
  }

  @Override
  public void resetPose2d(Pose2d pose) {
    odometry.resetPose(pose);
  }

  private Pose3d getPose() {
    return new Pose3d(odometry.getPoseMeters());
  }

  private ChassisSpeeds getRobotRelativeVelocities() {
    // TODO: add implementation
    return new ChassisSpeeds();
  }
}
