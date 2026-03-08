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

import java.util.ArrayList;
import java.util.List;
import org.austincans.lib.Constants;
import org.austincans.lib.motors.MotorIO;
import org.austincans.lib.subsystems.drive.ModuleIO;
import org.austincans.lib.util.swerve.ModuleLimits;
import org.austincans.lib.util.swerve.SwerveSetpoint;
import org.austincans.lib.util.swerve.SwerveSetpointGenerator;
import org.littletonrobotics.junction.Logger;
import edu.wpi.first.math.Vector;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.math.numbers.N2;

public abstract class DriveIOSwerve<T extends ModuleIO, U extends GyroIO> implements DriveIO {
  private final Translation2d[] moduleTranslations;
  private final SwerveDriveKinematics kinematics;
  private final SwerveSetpointGenerator swerveSetpointGenerator;

  // private final DriveState driveState;

  private final ArrayList<T> modules; // FL, FR, BL, FR // cannot have array of generics
  private final ModuleLimits moduleLimitsFree;

  private SwerveSetpoint currentSetpoint = new SwerveSetpoint(
    new ChassisSpeeds(),
    new SwerveModuleState[] {
      new SwerveModuleState(), 
      new SwerveModuleState(),
      new SwerveModuleState(),
      new SwerveModuleState()
    }
  );
  private boolean velocityMode = false;

  public DriveIOSwerve(DriveConfig driveConfig, Module frontLeft, Module frontRight, Module backLeft, Module backRight) {
    moduleTranslations = new Translation2d[] {
      new Translation2d(driveConfig.wheelbaseMeters() / 2, driveConfig.trackWidthMeters() / 2),
      new Translation2d(driveConfig.wheelbaseMeters() / 2, -driveConfig.trackWidthMeters() / 2),
      new Translation2d(-driveConfig.wheelbaseMeters() / 2, driveConfig.trackWidthMeters() / 2),
      new Translation2d(-driveConfig.wheelbaseMeters() / 2, -driveConfig.trackWidthMeters() / 2)
    };
    kinematics = new SwerveDriveKinematics(moduleTranslations);
    swerveSetpointGenerator = new SwerveSetpointGenerator(kinematics);

    modules = new ArrayList<T>(List.of(frontLeft, frontRight, backLeft, backRight));
    moduleLimitsFree = driveConfig.moduleConfig().moduleLimitsFree();
  }

  private SwerveModulePosition[] getModulePositions() {
    SwerveModulePosition[] positions = new SwerveModulePosition[4];
    for (int i = 0; i < 4; i++) {
      positions[i] = modules.get(i).getPosition();
    }
    return positions;
  }

  private SwerveModuleState[] getModuleStates() {
    SwerveModuleState[] states = new SwerveModuleState[4];
    for (int i = 0; i < 4; i++) {
      states[i] = modules.get(i).getState();
    }
    return states;
  }
  
  public void updateInputs(DriveIOInputs inputs) {
    if (!velocityMode) {
      currentSetpoint = new SwerveSetpoint(getVelocities(), getModuleStates()));
    }
    modules.get(0).updateInputs(inputs.frontLeftInputs);
    modules.get(1).updateInputs(inputs.frontRightInputs);
    modules.get(2).updateInputs(inputs.backLeftInputs);
    modules.get(3).updateInputs(inputs.backRightInputs);
  }

  /**
   * Runs the drive at the desired velocity.
   * @param speeds Speeds in meters/sec
   */
  @Override
  public void runVelocities(ChassisSpeeds velocities) {
    velocityMode = true;

    // Calculate module setpoints
    ChassisSpeeds discreteVelocities = ChassisSpeeds.discretize(velocities, Constants.loopPeriodSec);
    SwerveModuleState[] setpointStatesUnoptimized = kinematics.toSwerveModuleStates(discreteVelocities);
    currentSetpoint = swerveSetpointGenerator.generateSetpoint(
      moduleLimitsFree,
      currentSetpoint,
      discreteVelocities,
      Constants.loopPeriodSec);
    SwerveModuleState[] setpointStates = currentSetpoint.moduleStates();

    // Log unoptimized setpoints and setpoint speeds
    Logger.recordOutput("Drive/SwerveStates/SetpointsUnoptimized", setpointStatesUnoptimized);
    Logger.recordOutput("Drive/SwerveStates/Setpoints", setpointStates);
    Logger.recordOutput("Drive/SwerveChassisSpeeds/Setpoints", currentSetpoint.chassisSpeeds());

    // Send setpoints to modules
    for (int i = 0; i < 4; i++) {
      modules.get(i).runSetpoint(setpointStates[i]);
    }
  }

  /** Runs the drive in a straight line with the specified drive output. */
  public void runCharacterization(double output) {
    velocityMode = false;
    for (ModuleIO module : modules) {
      module.runCharacterization(output);
    }
  }

  /** Runs the drive in a straight line with the specified drive output. */
  public void runAngularCharacterization(double output) {
    velocityMode = false;
    for (ModuleIO module : modules) {
      module.runAngularCharacterization(output);
    }
  }

  /** Stops the drive. */
  public void stop() {
    runVelocities(new ChassisSpeeds());
  }

  /**
   * Stops the drive and turns the modules to an X arrangement to resist movement. The modules will
   * return to their normal orientations the next time a nonzero velocity is requested.
   */
  public void stopWithX() {
    Rotation2d[] headings = new Rotation2d[4];
    for (int i = 0; i < 4; i++) {
      headings[i] = moduleTranslations[i].getAngle();
    }
    kinematics.resetHeadings(headings);
    stop();
  }
  
  @Override
  public void resetPose2d(Pose2d pose) {

  }
}
