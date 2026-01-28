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
// Updated on Tuesday January 27, 2026 at 19:20:07 CST

package org.austincans.frc2158.subsystems.flywheel;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.math.system.plant.LinearSystemId;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.simulation.FlywheelSim;
import java.util.function.Supplier;
import org.austincans.frc2158.subsystems.sim.FlywheelSimViz3D;
import org.ironmaple.simulation.IntakeSimulation3D;
import org.ironmaple.simulation.ShooterSimulation3D;

public class FlywheelIOSim3D implements FlywheelIO {
  private final ShooterSimulation3D shooterSim;

  public FlywheelIOSim3D(ShooterSimulation3D shooterSim) {
    this.shooterSim = shooterSim;
  }

  @Override
  public void updateInputs(FlywheelIOInputs inputs) {
    inputs.velocityRPM = sim.getAngularVelocityRPM();
    inputs.appliedVolts = sim.getInputVoltage();

    // TODO: whether it can shoot should be based on whether it has passed through indexer
    // (create Indexer subsystem and IndexerViz)
    flywheelSimViz.update(inputs.velocityRPM, inputs.velocityRPM > 100);
  }

  @Override
  public void runVoltage(double volts) {}

  @Override
  public void runVelocity(double velocityRPM) {
    sim.setAngularVelocity(Units.rotationsToRadians(velocityRPM) / 60);
  }
}
