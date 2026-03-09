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

import org.austincans.lib.Constants;
import org.austincans.lib.configs.DriveConfigSwerve;
import org.austincans.lib.configs.ModuleConfig;
import org.austincans.lib.util.swerve.ModuleLimits;
import org.austincans.lib.util.swerve.SwerveSetpoint;
import org.austincans.lib.util.swerve.SwerveSetpointGenerator;
import org.littletonrobotics.junction.Logger;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;

// TODO: Maybe create Vision.java, a class for handling vision
public class DriveIOSwerve implements DriveIO {
  private final Translation2d[] moduleTranslations;
  private final SwerveDriveKinematics kinematics;
  private final SwerveSetpointGenerator swerveSetpointGenerator;
  private final ModuleLimits moduleLimitsFree;

  private final Module[] modules;
  private final Gyro gyro;

  private final SwerveDriveState swerveDriveState;

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

  public DriveIOSwerve(DriveConfigSwerve config, ModuleIO frontLeft, ModuleIO frontRight, ModuleIO backLeft, ModuleIO backRight, Gyro gyro) {
    ModuleConfig moduleConfig = config.moduleConfig;
    modules = new Module[] {
      new Module(frontLeft, moduleConfig), 
      new Module(frontRight, moduleConfig), 
      new Module(backLeft, moduleConfig), 
      new Module(backRight, moduleConfig)
    };
    this.gyro = gyro;

    moduleLimitsFree = config.moduleConfig.moduleLimitsFree();
    
    moduleTranslations = new Translation2d[] {
      new Translation2d(config.wheelbaseMeters / 2, config.trackWidthMeters / 2),
      new Translation2d(config.wheelbaseMeters / 2, -config.trackWidthMeters / 2),
      new Translation2d(-config.wheelbaseMeters / 2, config.trackWidthMeters / 2),
      new Translation2d(-config.wheelbaseMeters / 2, -config.trackWidthMeters / 2)
    };
    kinematics = new SwerveDriveKinematics(moduleTranslations);
    swerveSetpointGenerator = new SwerveSetpointGenerator(kinematics);

    swerveDriveState = new SwerveDriveState(kinematics);
  }

  /** Used for odometry */
  private SwerveModulePosition[] getModulePositions() {
    SwerveModulePosition[] positions = new SwerveModulePosition[4];
    for (int i = 0; i < 4; i++) {
      positions[i] = modules[i].getPosition();
    }
    return positions;
  }

  private SwerveModuleState[] getModuleStates() {
    SwerveModuleState[] states = new SwerveModuleState[4];
    for (int i = 0; i < 4; i++) {
      states[i] = modules[i].getState();
    }
    return states;
  }
  
  public void updateInputs(DriveIOInputs inputs) {
    for (Module module : modules) {
      module.periodic();
    }
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
      modules[i].runSetpoint(setpointStates[i]);
    }
  }

  /** Runs the drive in a straight line with the specified drive output. */
  public void runCharacterization(double output) {
    velocityMode = false;
    for (Module module : modules) {
      module.runCharacterization(output);
    }
  }

  /** Runs the drive in a straight line with the specified drive output. */
  public void runAngularCharacterization(double output) {
    velocityMode = false;
    for (int i = 0; i < 4; i++) {
      modules[i].runAngularCharacterization(output, moduleTranslations[i]);
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
    swerveDriveState.resetPose(pose);
  }
}
