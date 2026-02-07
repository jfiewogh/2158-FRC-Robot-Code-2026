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
// Updated on Thursday February 05, 2026 at 19:51:21 CST

package org.austincans.frc2158.subsystems.flywheel;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;

import org.austincans.frc2158.controllers.BlinkinLEDController;
import org.austincans.frc2158.controllers.LaserCAN;
import org.austincans.frc2158.subsystems.selfcheck.AdvancedSubsystem;
import org.littletonrobotics.junction.Logger;

/*
 * In statistics and control theory, Kalman filtering (also known as linear quadratic estimation)
 * is an algorithm that uses a series of measurements observed over time, including statistical
 * noise and other inaccuracies, to produce estimates of unknown variables that tend to be more
 * accurate than those based on a single measurement, by estimating a joint probability
 * distribution over the variables for each time-step.
 * The filter is constructed as a mean squared error minimiser, but an alternative derivation of
 * the filter is also provided showing how the filter relates to maximum likelihood statistics.
 * https://docs.wpilib.org/en/stable/docs/software/advanced-controls/state-space/state-space-observers.html#kalman-filters
 */
public class Flywheel extends AdvancedSubsystem {
  private final FlywheelIO io;
  private final FlywheelIOInputsAutoLogged inputs = new FlywheelIOInputsAutoLogged();

  private LaserCAN lasarCan1 = new LaserCAN(0);
  private LaserCAN laserCan2 = new LaserCAN(1);

  /** Creates a new Flywheel. */
  public Flywheel(FlywheelIO io) {
    this.io = io;
  }

  public boolean readyToShoot(){
    return lasarCan1.getMeasurement().distance_mm < FlywheelConstants.maxDistFromLasarCan1;
  }

  public boolean isShooting(){
    return laserCan2.getMeasurement().distance_mm < FlywheelConstants.maxDistFromLasarCan2;
  }

  @Override
  public void periodic() {
    io.updateInputs(inputs);
    Logger.processInputs("Flywheel", inputs);

    BlinkinLEDController.getInstance().readyToShoot = readyToShoot();  
    BlinkinLEDController.getInstance().isShooting = isShooting();
  }

  public void setVelocity(double velocityRPM) {
    io.runVelocity(velocityRPM);
  }

  public double getRPM() {
    return inputs.velocityRPM;
  }

  @Override
  public Command systemCheckCommand() {
    return Commands.sequence(
        Commands.runOnce(() -> io.runVelocity(100)),
        Commands.waitSeconds(0.3),
        Commands.runOnce(
            () -> {
              if (getRPM() < 95 || getRPM() > 105) {
                addFault("[System Check] Flywheel did not reach desired velocity RPM");
              }
            }));
  }
}
