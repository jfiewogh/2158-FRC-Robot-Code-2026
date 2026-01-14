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
// Updated on Tuesday January 13, 2026 at 18:29:34 CST

package org.austincans.frc2158.subsystems.flywheel;

import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.math.system.plant.LinearSystemId;
import edu.wpi.first.wpilibj.simulation.FlywheelSim;
import org.austincans.frc2158.Constants;

public class FlywheelIOSim implements FlywheelIO {
  private final DCMotor plant = DCMotor.getNeoVortex(1);
  private final FlywheelSim sim =
      new FlywheelSim(
          LinearSystemId.createFlywheelSystem(
              plant, FlywheelConstants.JKgMetersSq, FlywheelConstants.gearing),
          plant);

  private double appliedVolts = 0;

  @Override
  public void updateInputs(FlywheelIOInputs inputs) {
    sim.setInputVoltage(appliedVolts);
    sim.update(Constants.loopPeriodSecs);

    inputs.appliedVolts = appliedVolts;
    inputs.velocityRPM = sim.getAngularVelocityRPM();
    inputs.temperatureC = 0;
  }

  @Override
  public void runVoltage(double volts) {
    appliedVolts = volts;
  }
}
