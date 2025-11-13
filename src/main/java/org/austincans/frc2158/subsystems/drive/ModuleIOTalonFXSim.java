// Copyright (c) 2025 the ausTIN CANs
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
// Updated on Thursday November 13, 2025 at 14:16:40 CST

package org.austincans.frc2158.subsystems.drive;

import static edu.wpi.first.units.Units.*;

import com.ctre.phoenix6.swerve.SwerveModuleConstants;
import java.util.Arrays;
import org.austincans.frc2158.util.PhoenixUtil;
import org.ironmaple.simulation.drivesims.SwerveModuleSimulation;

/**
 * Physics sim implementation of module IO. The sim models are configured using a set of module
 * constants from Phoenix. Simulation is always based on voltage control.
 */
public class ModuleIOTalonFXSim extends ModuleIOTalonFX {
  private final SwerveModuleSimulation simulation;

  public ModuleIOTalonFXSim(SwerveModuleConstants constants, SwerveModuleSimulation simulation) {
    super(PhoenixUtil.regulateModuleConstantForSimulation(constants));

    this.simulation = simulation;
    simulation.useDriveMotorController(new PhoenixUtil.TalonFXMotorControllerSim(driveTalon));

    simulation.useSteerMotorController(
        new PhoenixUtil.TalonFXMotorControllerWithRemoteCancoderSim(turnTalon, cancoder));
  }

  @Override
  public void updateInputs(ModuleIOInputs inputs) {
    super.updateInputs(inputs);

    // Update odometry inputs
    inputs.odometryTimestamps = PhoenixUtil.getSimulationOdometryTimeStamps();

    inputs.odometryDrivePositionsRad =
        Arrays.stream(simulation.getCachedDriveWheelFinalPositions())
            .mapToDouble(angle -> angle.in(Radians))
            .toArray();

    inputs.odometryTurnPositions = simulation.getCachedSteerAbsolutePositions();
  }
}
