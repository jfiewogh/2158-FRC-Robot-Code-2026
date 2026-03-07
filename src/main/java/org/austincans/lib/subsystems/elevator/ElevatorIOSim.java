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

package org.austincans.lib.subsystems.elevator;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.wpilibj.simulation.ElevatorSim;
import org.austincans.frc2026.Constants;
import org.austincans.frc2026.subsystems.sim.ElevatorViz;

public class ElevatorIOSim implements ElevatorIO {
  private final DCMotor motorModel = DCMotor.getNeoVortex(1);

  private final ElevatorSim elevatorSim =
      new ElevatorSim(
          motorModel, // Motor type
          ElevatorConstants.gearRatio,
          5, // Carriage mass (kg)
          ElevatorConstants.drumRadius, // Drum radius (m)
          ElevatorConstants.minHeightMeters, // Min height (m)
          ElevatorConstants.maxHeightMeters, // Max height (m)
          true, // Simulate gravity
          0 // Starting height (m)
          );

  private final ElevatorViz elevatorViz;

  private double appliedVolts;

  public ElevatorIOSim() {
    elevatorViz = new ElevatorViz("SimComponents", 0);
  }

  @Override
  public void updateInputs(ElevatorIOInputs inputs) {
    elevatorSim.setInputVoltage(appliedVolts);
    elevatorSim.update(Constants.loopPeriodSecs);

    inputs.connected = false;
    inputs.heightMeters = elevatorSim.getPositionMeters();
    inputs.velocityMetersPerSec = elevatorSim.getVelocityMetersPerSecond();
    inputs.voltage = appliedVolts;
    inputs.current = elevatorSim.getCurrentDrawAmps();
    inputs.temperatureC = 0.0;

    elevatorViz.update(inputs.heightMeters);
  }

  @Override
  public void runVoltage(double volts) {
    appliedVolts = MathUtil.clamp(volts, -12.0, 12.0);
  }
}
