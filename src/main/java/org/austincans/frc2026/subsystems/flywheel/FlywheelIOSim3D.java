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

package org.austincans.frc2026.subsystems.flywheel;

import edu.wpi.first.math.geometry.Rotation2d;
import java.util.function.Supplier;
import org.austincans.frc2026.simulation.RobotSimulation3D;
import org.austincans.frc2026.subsystems.sim.ShooterSim3D;
import org.ironmaple.simulation.IntakeSimulation3D;
import org.ironmaple.simulation.drivesims.SwerveDriveSimulation3D;

public class FlywheelIOSim3D implements FlywheelIO {
  private final ShooterSim3D shooterSim;
  private final IntakeSimulation3D intakeSimulation;

  private double velocityRPM;

  public FlywheelIOSim3D(
      SwerveDriveSimulation3D driveSimulation,
      IntakeSimulation3D intakeSimulation,
      Supplier<Rotation2d> hoodAngleSupplier,
      Supplier<Rotation2d> turretAngle,
      RobotSimulation3D robotSimulation) {
    shooterSim =
        new ShooterSim3D(
            FlywheelConstants.radiusMeters,
            0.1,
            driveSimulation,
            intakeSimulation,
            hoodAngleSupplier,
            turretAngle,
            robotSimulation);
    this.intakeSimulation = intakeSimulation;
  }

  @Override
  public void updateInputs(FlywheelIOInputs inputs) {
    inputs.velocityRPM = velocityRPM;
    shooterSim.update(velocityRPM);
  }

  @Override
  public void runRelativeVelocity(double velocityRPM) {
    this.velocityRPM = velocityRPM;
  }

  @Override
  public boolean readyToShoot() {
    return intakeSimulation.getGamePiecesInIntake() > 0;
  }

  @Override
  public boolean isShooting() {
    return velocityRPM > 100 && readyToShoot();
  }

  @Override
  public boolean noGamePiecesToShoot() {
    System.out.println(intakeSimulation.getGamePiecesInIntake());
    return intakeSimulation.getGamePiecesInIntake() == 0;
  }
}
