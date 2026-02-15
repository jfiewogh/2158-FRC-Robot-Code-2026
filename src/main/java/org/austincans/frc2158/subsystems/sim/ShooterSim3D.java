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
// Updated on Sunday February 15, 2026 at 10:53:53 CST

package org.austincans.frc2158.subsystems.sim;

import static edu.wpi.first.units.Units.MetersPerSecond;
import static edu.wpi.first.units.Units.Radians;

import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.LinearVelocity;
import edu.wpi.first.wpilibj.Timer;
import java.util.function.Supplier;
import org.austincans.frc2158.subsystems.turret.TurretConstants;
import org.ironmaple.simulation.IntakeSimulation3D;
import org.ironmaple.simulation.ShooterSimulation3D;
import org.ironmaple.simulation.SimulatedArena3D;
import org.ironmaple.simulation.drivesims.SwerveDriveSimulation3D;
import org.ironmaple.simulation.gamepieces.FuelBall;

public class ShooterSim3D {
  private final double radius;
  private final double shootingDelay;

  private final Supplier<Rotation2d> hoodAngleSupplier;

  private final Timer timer = new Timer();
  private double lastShootingTime = 0;

  private final ShooterSimulation3D shooterSim;

  public ShooterSim3D(
      double radius,
      double shootingDelay,
      SwerveDriveSimulation3D driveSimulation,
      IntakeSimulation3D intakeSimulation,
      Supplier<Rotation2d> hoodAngleSupplier,
      Supplier<Rotation2d> turretAngleSupplier) {
    this.radius = radius;
    this.shootingDelay = shootingDelay;
    this.hoodAngleSupplier = hoodAngleSupplier;

    Supplier<Pose3d> shooterPose =
        () ->
            new Pose3d(
                new Translation3d(0, 0, TurretConstants.turretHeight),
                new Rotation3d(
                    0,
                    0,
                    turretAngleSupplier.get().getRadians()
                        - driveSimulation.getGyroSimulation().getGyroReading().getRadians()));

    shooterSim =
        ShooterSimulation3D.create(FuelBall.createInfo(SimulatedArena3D.getInstance()))
            .onRobot(driveSimulation, shooterPose)
            .withCapacity(1)
            .withIntakeSource(intakeSimulation)
            .build();

    timer.start();
  }

  public void update(double velocityRPM) {
    if ((Timer.getFPGATimestamp() - lastShootingTime) > shootingDelay && velocityRPM > 100) {

      if (shooterSim.hasGamePiece()) {
        lastShootingTime = Timer.getFPGATimestamp();

        double exitVelocityMetersPerSec = Units.rotationsToRadians(velocityRPM) / 60 * radius;

        LinearVelocity launchingSpeed =
            LinearVelocity.ofBaseUnits(exitVelocityMetersPerSec, MetersPerSecond);
        Angle shooterAngle = Angle.ofBaseUnits(hoodAngleSupplier.get().getRadians(), Radians);

        shooterSim.shoot(launchingSpeed, shooterAngle);

        // System.out.println("SHOT at " + exitVelocityMetersPerSec);
      }
    }
  }
}
