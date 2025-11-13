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

package org.austincans.frc2158.subsystems.turret;

import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.smartdashboard.Mechanism2d;
import edu.wpi.first.wpilibj.smartdashboard.MechanismLigament2d;
import edu.wpi.first.wpilibj.smartdashboard.MechanismRoot2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj.util.Color;
import edu.wpi.first.wpilibj.util.Color8Bit;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

/** Visualization for the pivot subsystem in simulation. */
public class TurretSubsystemSim extends SubsystemBase {

  private final TurretSubsystem pivot;

  // Simulation display
  private final Mechanism2d mech;
  private final MechanismRoot2d root;
  private final MechanismLigament2d pivotMech;

  // Visualization constants
  private final double BASE_WIDTH = 60.0;
  private final double BASE_HEIGHT = 10.0;
  private final double PIVOT_LENGTH = 40.0;
  private final double PIVOT_WIDTH = 6.0;

  /**
   * Creates a new visualization for the pivot.
   *
   * @param pivotSubsystem The pivot subsystem to visualize
   */
  public TurretSubsystemSim(TurretSubsystem pivotSubsystem) {
    this.pivot = pivotSubsystem;

    // Create the simulation display
    mech = new Mechanism2d(300, 300);
    root = mech.getRoot("PivotRoot", 150, 150);

    // Add base
    MechanismLigament2d base =
        root.append(
            new MechanismLigament2d(
                "Base", BASE_WIDTH, 0, BASE_HEIGHT, new Color8Bit(Color.kDarkGray)));

    // Add pivot point
    MechanismLigament2d pivotPoint =
        base.append(new MechanismLigament2d("PivotPoint", 5, 90, 5, new Color8Bit(Color.kBlack)));

    // Add the pivot mechanism
    pivotMech =
        pivotPoint.append(
            new MechanismLigament2d(
                "Pivot", PIVOT_LENGTH, 0, PIVOT_WIDTH, new Color8Bit(Color.kBlue)));

    // Initialize visualization
    SmartDashboard.putData("Pivot Sim", mech);
  }

  @Override
  public void periodic() {
    // Update pivot angle
    double currentAngleRad = pivot.getSimulation().getAngleRads();
    pivotMech.setAngle(Units.radiansToDegrees(currentAngleRad));

    // Add telemetry data
    SmartDashboard.putNumber("Pivot Angle (deg)", Units.radiansToDegrees(currentAngleRad));
    SmartDashboard.putNumber(
        "Pivot Velocity (deg/s)",
        Units.radiansToDegrees(pivot.getSimulation().getVelocityRadPerSec()));
    SmartDashboard.putNumber("Pivot Current (A)", pivot.getSimulation().getCurrentDrawAmps());
  }
}
