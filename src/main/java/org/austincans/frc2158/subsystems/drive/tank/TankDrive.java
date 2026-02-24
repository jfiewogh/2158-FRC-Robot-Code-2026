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
// Updated on Saturday February 21, 2026 at 16:51:46 CST

package org.austincans.frc2158.subsystems.drive.tank;

import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.DifferentialDriveKinematics;
import edu.wpi.first.math.kinematics.DifferentialDriveWheelSpeeds;
import edu.wpi.first.wpilibj.motorcontrol.PWMMotorController;
import java.util.function.DoubleSupplier;
import org.austincans.frc2158.subsystems.drive.DriveConstants;

public class TankDrive {
  private static final DifferentialDriveKinematics kinematics =
      new DifferentialDriveKinematics(DriveConstants.trackWidthY);

  private PWMMotorController[] motors =
      new PWMMotorController[] {
        new PWMMotorController("frontLeft", 0) {},
        new PWMMotorController("frontRight", 1) {},
        new PWMMotorController("backLeft", 2) {},
        new PWMMotorController("backRight", 3) {},
      };

  // private final DifferentialDriveOdometry odometry = new DifferentialDriveOdometry(
  //     Rotation2d.kZero, motors[0].
  // );

  public TankDrive() {
    // RobotConfig config;
    // try{
    //     config = RobotConfig.fromGUISettings();
    // } catch (Exception e) {
    //     // Handle exception as needed
    //     e.printStackTrace();
    // }

    // AutoBuilder.configure(
    //     this::getPose, // Robot pose supplier
    //     this::resetPose, // Method to reset odometry (will be called if your auto has a starting
    // pose)
    //     this::getRobotRelativeSpeeds, // ChassisSpeeds supplier. MUST BE ROBOT RELATIVE
    //     (speeds, feedforwards) -> runChassisSpeeds(speeds), // Method that will drive the robot
    // given ROBOT RELATIVE ChassisSpeeds. Also optionally outputs individual module feedforwards
    //     new PPLTVController(0.02), // PPLTVController is the built in path following controller
    // for differential drive trains
    //     config, // The robot configuration
    //     () -> {
    //       // Boolean supplier that controls when the path will be mirrored for the red alliance
    //       // This will flip the path being followed to the red side of the field.
    //       // THE ORIGIN WILL REMAIN ON THE BLUE SIDE

    //       var alliance = DriverStation.getAlliance();
    //       if (alliance.isPresent()) {
    //         return alliance.get() == DriverStation.Alliance.Red;
    //       }
    //       return false;
    //     },
    //     this // Reference to this subsystem to set requirements
    // );
  }

  public void runChassisSpeeds(ChassisSpeeds speeds) {
    DifferentialDriveWheelSpeeds wheelSpeeds = kinematics.toWheelSpeeds(speeds);
    double leftVelocity =
        Math.min(
            wheelSpeeds.leftMetersPerSecond / DriveConstants.maxLinearSpeed,
            DriveConstants.maxLinearSpeed);
    double rightVelocity =
        Math.min(
            wheelSpeeds.rightMetersPerSecond / DriveConstants.maxLinearSpeed,
            DriveConstants.maxLinearSpeed);
    motors[0].set(leftVelocity);
    motors[1].set(leftVelocity);
    motors[2].set(rightVelocity);
    motors[3].set(rightVelocity);
  }

  public void arcadeDrive(DoubleSupplier xInput, DoubleSupplier omegaInput) {
    runChassisSpeeds(new ChassisSpeeds(xInput.getAsDouble(), 0, omegaInput.getAsDouble()));
  }

  // public Pose2d getPose() {
  //     return
  // }

  // public Pose2d resetPose() {

  // }

  // public ChassisSpeeds getRobotRelativeSpeeds() {
  //     return
  // }
}
