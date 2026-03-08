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
import java.util.function.DoubleSupplier;

import org.austincans.lib.configs.DriveConfig;
import org.austincans.lib.motors.MotorIO;

import com.pathplanner.lib.config.RobotConfig;

// TODO: Finish this class
public class DriveIODifferential<T extends MotorIO, U extends GyroIO> implements DriveIO {
  private final DifferentialDriveKinematics kinematics;

  private final T leftLeader;
  private final T leftFollower;
  private final T rightLeader;
  private final T rightFollower;

  private final U gyroIO;

  private final DifferentialDriveOdometry odometry;

  public DriveIODifferential(DriveConfig driveConfig, T leftLeader, T leftFollower, T rightLeader, T rightFollower, U gyroIO) {
    kinematics = new DifferentialDriveKinematics(driveConfig.trackWidthMeters());

    this.leftLeader = leftLeader;
    this.leftFollower = leftFollower;
    this.rightLeader = rightLeader;
    this.rightFollower = rightFollower;

    this.gyroIO = gyroIO;

    odometry = new DifferentialDriveOdometry(gyroIO., null, null);

  }

  @Override
  public void updateInputs(DriveIOInputs inputs) {
    inputs.pose = getPose();
    inputs.velocities = getRobotRelativeVelocities();
  }

  @Override
  public void runVelocities(ChassisSpeeds velocities) {
    DifferentialDriveWheelSpeeds wheelVelocities = kinematics.toWheelSpeeds(velocities);
    double leftVelocity = Math.min(
        wheelVelocities.leftMetersPerSecond / DriveConstants.maxLinearSpeed,
        DriveConstants.maxLinearSpeed);
    double rightVelocity = Math.min(
        wheelVelocities.rightMetersPerSecond / DriveConstants.maxLinearSpeed,
        DriveConstants.maxLinearSpeed);
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
