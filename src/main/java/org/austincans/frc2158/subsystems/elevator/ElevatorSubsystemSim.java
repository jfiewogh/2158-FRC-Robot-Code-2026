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
// Updated on Thursday November 13, 2025 at 17:43:29 CST

package org.austincans.frc2158.subsystems.elevator;

import edu.wpi.first.wpilibj.smartdashboard.Mechanism2d;
import edu.wpi.first.wpilibj.smartdashboard.MechanismLigament2d;
import edu.wpi.first.wpilibj.smartdashboard.MechanismRoot2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj.util.Color;
import edu.wpi.first.wpilibj.util.Color8Bit;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

/** Visualization for the elevator subsystem in simulation. */
public class ElevatorSubsystemSim extends SubsystemBase {

  private final ElevatorSubsystem elevator;

  // Simulation display
  private final Mechanism2d mech;
  private final MechanismRoot2d root;
  private final MechanismLigament2d elevatorMech;

  // Visualization constants
  private final double VISUAL_WIDTH = 10.0; // Width of visualization in pixels
  private final double BASE_HEIGHT = 20.0; // Height of base in pixels
  private final double CARRIAGE_WIDTH = 30.0; // Width of elevator carriage in pixels

  @SuppressWarnings("unused")
  private final double CARRIAGE_HEIGHT = 40.0; // Height of elevator carriage in pixels

  // Elevator parameters
  private final double minHeight;
  private final double maxHeight;
  private final double visualScaleFactor;

  /**
   * Creates a new visualization for the elevator.
   *
   * @param elevatorSubsystem The elevator subsystem to visualize
   */
  public ElevatorSubsystemSim(ElevatorSubsystem elevatorSubsystem) {
    this.elevator = elevatorSubsystem;

    // Get elevator parameters from simulation
    minHeight = elevator.getMinHeightMeters();
    maxHeight = elevator.getMaxHeightMeters();

    // Calculate scale factor to keep visualization in reasonable bounds
    double elevatorTravel = maxHeight - minHeight;
    visualScaleFactor = 300.0 / elevatorTravel; // Scale to ~300 pixels max

    // Create the simulation display
    mech = new Mechanism2d(400, 400);
    root = mech.getRoot("ElevatorRoot", 200, 50);

    // Add elevator base
    MechanismLigament2d elevatorBase =
        root.append(
            new MechanismLigament2d("Base", BASE_HEIGHT, 90, 6, new Color8Bit(Color.kDarkGray)));

    // Add elevator tower
    @SuppressWarnings("unused")
    MechanismLigament2d elevatorTower =
        elevatorBase.append(
            new MechanismLigament2d(
                "Tower",
                (maxHeight - minHeight) * visualScaleFactor,
                90,
                VISUAL_WIDTH,
                new Color8Bit(Color.kGray)));

    // Add elevator carriage
    elevatorMech =
        root.append(
            new MechanismLigament2d(
                "Elevator", BASE_HEIGHT, 90, CARRIAGE_WIDTH, new Color8Bit(Color.kBlue)));

    // Initialize visualization
    SmartDashboard.putData("Elevator Sim", mech);
  }

  @Override
  public void periodic() {
    // Update elevator height
    double currentHeight = elevator.getSimulation().getPositionMeters();
    double displayHeight = BASE_HEIGHT + (currentHeight - minHeight) * visualScaleFactor;
    elevatorMech.setLength(displayHeight);

    // Add telemetry data
    SmartDashboard.putNumber("Elevator Height (m)", currentHeight);
    SmartDashboard.putNumber(
        "Elevator Velocity (m/s)", elevator.getSimulation().getVelocityMetersPerSecond());
    SmartDashboard.putNumber("Elevator Current (A)", elevator.getSimulation().getCurrentDrawAmps());
  }
}
