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

package org.austincans.lib.subsystems.drive.differential;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.DifferentialDriveKinematics;
import edu.wpi.first.math.kinematics.DifferentialDriveOdometry;
import edu.wpi.first.math.kinematics.DifferentialDriveWheelSpeeds;
import java.util.function.DoubleSupplier;
import org.austincans.lib.motors.MotorIO;
import org.austincans.lib.subsystems.drive.DriveConfig;
import org.austincans.lib.subsystems.drive.DriveIO;
import com.pathplanner.lib.config.RobotConfig;

public class DriveIODifferential implements DriveIO {
  private final DifferentialDriveKinematics kinematics;

  private final MotorIO leftLeader;
  private final MotorIO leftFollower;
  private final MotorIO rightLeader;
  private final MotorIO rightFollower;

  private final DifferentialDriveOdometry odometry;

  public DriveIODifferential(DriveConfig driveConfig, MotorIO leftLeader, MotorIO leftFollower,
      MotorIO rightLeader, MotorIO rightFollower) {
    kinematics = new DifferentialDriveKinematics(driveConfig.trackWidthMeters());

    this.leftLeader = leftLeader;
    this.leftFollower = leftFollower;
    this.rightLeader = rightLeader;
    this.rightFollower = rightFollower;
    // TODO: Add set follower to MotorIO

    odometry = new DifferentialDriveOdometry(null, null, null);

    RobotConfig robotConfig;
    try {
      robotConfig = RobotConfig.fromGUISettings();
    } catch (Exception e) {
      // Handle exception as needed
      e.printStackTrace();
    }

    // TODO: AutoBuilder.configure must take in the subsystem. Fix this
    /*
    AutoBuilder.configure(
        this::getPose, // Robot pose supplier
        this::resetPose, // Method to reset odometry (will be called if your auto has a starting pose)
        this::getRobotRelativeSpeeds, // ChassisSpeeds supplier. MUST BE ROBOT RELATIVE
        (speeds, feedforwards) -> runChassisSpeeds(speeds), // Method that will drive the robot given ROBOT RELATIVE
                                                            // ChassisSpeeds. Also optionally outputs individual module
                                                            // feedforwards
        new PPLTVController(0.02), // PPLTVController is the built in path following controller for differential
                                   // drive trains
        robotConfig, // The robot configuration
        () -> {
          // Boolean supplier that controls when the path will be mirrored for the red
          // alliance
          // This will flip the path being followed to the red side of the field.
          // THE ORIGIN WILL REMAIN ON THE BLUE SIDE
          var alliance = DriverStation.getAlliance();
          if (alliance.isPresent()) {
            return alliance.get() == DriverStation.Alliance.Red;
          }
          return false;
        },
        this // Reference to this subsystem to set requirements
    );
    */
  }

  @Override
  public void updateInputs(DriveIOInputs inputs) {
    inputs.pose = getPose();
    inputs.velocities = getRobotRelativeVelocities();
  }

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

  private Pose3d getPose() {
    return new Pose3d(odometry.getPoseMeters());
  }

  private void resetPose(Pose2d pose) {}

  private ChassisSpeeds getRobotRelativeVelocities() {
    // TODO: add implementation
    return new ChassisSpeeds();
  }
}
