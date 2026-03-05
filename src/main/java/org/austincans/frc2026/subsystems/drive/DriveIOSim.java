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

package org.austincans.frc2026.subsystems.drive;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import org.austincans.frc2026.Constants;
import org.austincans.frc2026.util.swerve.SwerveSetpoint;
import org.ironmaple.simulation.drivesims.SwerveDriveSimulation;
import org.littletonrobotics.junction.Logger;

// TODO: Drive simulation is affected by other simulation components attached to it, which decrease
// its speed
// Maybe update the PID and feedback controllers to take this into account
public class DriveIOSim implements DriveIO {
  private final SwerveDriveSimulation simulation;

  private Module[] modules;

  private SwerveSetpoint currentSetpoint;

  public DriveIOSim(SwerveDriveSimulation simulation) {
    this.simulation = simulation;
  }

  @Override
  public void updateInputs(DriveIOInputs inputs) {
    inputs.pose = new Pose3d(simulation.getSimulatedDriveTrainPose());
    inputs.chassisSpeeds = simulation.getDriveTrainSimulatedChassisSpeedsFieldRelative();
  }

  @Override
  public void runVelocity(ChassisSpeeds speeds) {
    // Calculate module setpoints
    ChassisSpeeds discreteSpeeds = ChassisSpeeds.discretize(speeds, Constants.loopPeriodSecs);
    SwerveModuleState[] setpointStatesUnoptimized = kinematics.toSwerveModuleStates(discreteSpeeds);
    currentSetpoint =
        swerveSetpointGenerator.generateSetpoint(
            DriveConstants.moduleLimitsFree,
            currentSetpoint != null
                ? currentSetpoint
                : new SwerveSetpoint(new ChassisSpeeds(), getModuleStates()),
            discreteSpeeds,
            Constants.loopPeriodSecs);
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

  @Override
  public void setModules(Module[] modules) {
    this.modules = modules;
  }

  @Override
  public void resetPose(Pose2d pose) {
    simulation.setSimulationWorldPose(pose);
  }

  private SwerveModuleState[] getModuleStates() {
    SwerveModuleState[] states = new SwerveModuleState[4];
    for (int i = 0; i < 4; i++) {
      states[i] = modules[i].getState();
    }
    return states;
  }
}
