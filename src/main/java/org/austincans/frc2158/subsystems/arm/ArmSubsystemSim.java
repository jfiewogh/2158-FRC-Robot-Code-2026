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
// Updated on Wednesday November 05, 2025 at 17:57:47 CST

package org.austincans.frc2158.subsystems.arm;

import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.smartdashboard.Mechanism2d;
import edu.wpi.first.wpilibj.smartdashboard.MechanismLigament2d;
import edu.wpi.first.wpilibj.smartdashboard.MechanismRoot2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj.util.Color;
import edu.wpi.first.wpilibj.util.Color8Bit;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

/** Visualization for the arm subsystem in simulation. */
public class ArmSubsystemSim extends SubsystemBase {

  private final ArmSubsystem arm;

  // Simulation display
  private final Mechanism2d mech;
  private final MechanismRoot2d root;
  private final MechanismLigament2d armMech;

  // Visualization constants
  private final double BASE_WIDTH = 40.0;
  private final double BASE_HEIGHT = 20.0;
  private final double TOWER_HEIGHT = 30.0;
  private final double ARM_WIDTH = 10.0;

  // Arm parameters
  private final double armLength;
  private final double visualScaleFactor;

  /**
   * Creates a new visualization for the arm.
   *
   * @param armSubsystem The arm subsystem to visualize
   */
  public ArmSubsystemSim(ArmSubsystem armSubsystem) {
    this.arm = armSubsystem;

    // Get arm length from subsystem (in meters)
    armLength = 1;

    // Calculate scale factor to keep visualization in reasonable bounds
    visualScaleFactor = 200.0 / armLength; // Scale to ~200 pixels

    // Create the simulation display
    mech = new Mechanism2d(400, 400);
    root = mech.getRoot("ArmRoot", 200, 200);

    // Add arm base
    MechanismLigament2d armBase =
        root.append(
            new MechanismLigament2d(
                "Base", BASE_WIDTH, 0, BASE_HEIGHT, new Color8Bit(Color.kDarkGray)));

    // Add tower
    MechanismLigament2d tower =
        armBase.append(
            new MechanismLigament2d(
                "Tower", TOWER_HEIGHT, 90, BASE_HEIGHT / 2, new Color8Bit(Color.kGray)));

    // Add the arm pivot point
    MechanismLigament2d pivot =
        tower.append(new MechanismLigament2d("Pivot", 5, 0, 5, new Color8Bit(Color.kBlack)));

    // Add the arm
    armMech =
        pivot.append(
            new MechanismLigament2d(
                "Arm", armLength * visualScaleFactor, 0, ARM_WIDTH, new Color8Bit(Color.kBlue)));

    // Initialize visualization
    SmartDashboard.putData("Arm Sim", mech);
  }

  @Override
  public void periodic() {
    // Update arm angle
    double currentAngleRad = arm.getSimulation().getAngleRads();
    armMech.setAngle(Units.radiansToDegrees(currentAngleRad));

    // Add telemetry data
    SmartDashboard.putNumber("Arm Angle (deg)", Units.radiansToDegrees(currentAngleRad));
    SmartDashboard.putNumber(
        "Arm Velocity (deg/s)", Units.radiansToDegrees(arm.getSimulation().getVelocityRadPerSec()));
    SmartDashboard.putNumber("Arm Current (A)", arm.getSimulation().getCurrentDrawAmps());
  }
}
