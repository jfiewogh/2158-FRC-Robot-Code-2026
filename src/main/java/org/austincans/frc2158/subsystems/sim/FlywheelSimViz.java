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
// Updated on Saturday January 17, 2026 at 11:21:17 CST

package org.austincans.frc2158.subsystems.sim;

import static edu.wpi.first.units.Units.Degrees;
import static edu.wpi.first.units.Units.Meters;
import static edu.wpi.first.units.Units.MetersPerSecond;

import java.util.function.DoubleSupplier;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.Distance;
import edu.wpi.first.units.measure.LinearVelocity;
import edu.wpi.first.wpilibj.Timer;
import org.austincans.frc2158.RobotState;
import org.ironmaple.simulation.IntakeSimulation;
import org.ironmaple.simulation.SimulatedArena;
import org.ironmaple.simulation.seasonspecific.rebuilt2026.RebuiltFuelOnFly;

// This simulation handles shooting fuel
public class FlywheelSimViz {
  private final double radius;
  private final double shootingDelay;
  private final IntakeSimulation intakeSimulation;

  private final Timer timer = new Timer();
  private double lastShootingTime = 0;

  public FlywheelSimViz(double radius, double shootingDelay, 
    IntakeSimulation intakeSimulation, DoubleSupplier hoodAngleRadSupplier) {
    this.radius = radius;
    this.shootingDelay = shootingDelay;
    this.intakeSimulation = intakeSimulation;
    timer.start();
  }

  public void update(double velocityRPM, boolean canShoot) {
    if (canShoot
        && (Timer.getFPGATimestamp() - lastShootingTime) > shootingDelay
        && intakeSimulation.obtainGamePieceFromIntake()) {
      lastShootingTime = Timer.getFPGATimestamp();

      double exitVelocityMetersPerSec = Units.rotationsToRadians(velocityRPM) / 60 * radius;

      Pose2d robotPose = RobotState.getInstance().getEstimatedPose();

      Distance initialHeight = Distance.ofBaseUnits(0.5, Meters);
      LinearVelocity launchingSpeed =
          LinearVelocity.ofBaseUnits(exitVelocityMetersPerSec, MetersPerSecond);
      Angle shooterAngle = Angle.ofBaseUnits(70, Degrees); // TODO: should be based on hood angle

      SimulatedArena.getInstance()
          .addGamePieceProjectile(
              new RebuiltFuelOnFly(
                  robotPose.getTranslation(),
                  new Translation2d(),
                  RobotState.getInstance().getFieldVelocity(),
                  robotPose.getRotation(),
                  initialHeight,
                  launchingSpeed,
                  shooterAngle));
    }
  }
}
