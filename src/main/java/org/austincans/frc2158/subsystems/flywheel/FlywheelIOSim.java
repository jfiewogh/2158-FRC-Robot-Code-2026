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
// Updated on Saturday January 31, 2026 at 16:59:54 CST

package org.austincans.frc2158.subsystems.flywheel;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.math.system.plant.LinearSystemId;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.simulation.FlywheelSim;
import java.util.function.Supplier;
import org.austincans.frc2158.subsystems.sim.ShooterSim;
import org.ironmaple.simulation.IntakeSimulation;

public class FlywheelIOSim implements FlywheelIO {
  private final ShooterSim shooterSim;

  private final DCMotor plant = DCMotor.getNeoVortex(1);
  private final FlywheelSim sim =
      new FlywheelSim(
          LinearSystemId.createFlywheelSystem(
              plant, FlywheelConstants.JKgMetersSq, FlywheelConstants.gearing),
          plant);

  private final PIDController controller = new PIDController(5, 0, 0);
  private final SimpleMotorFeedforward feedforwardController =
      new SimpleMotorFeedforward(FlywheelConstants.kS, FlywheelConstants.kV, FlywheelConstants.kA);

  private double appliedVolts;

  public FlywheelIOSim(
      IntakeSimulation intakeSimulation,
      Supplier<Rotation2d> hoodAngleSupplier,
      Supplier<Rotation2d> turretAngle) {
    shooterSim =
        new ShooterSim(
            FlywheelConstants.radiusMeters, 0.1, intakeSimulation, hoodAngleSupplier, turretAngle);
  }

  @Override
  public void updateInputs(FlywheelIOInputs inputs) {
    inputs.velocityRPM = sim.getAngularVelocityRPM();

    // double feedback =
    //     controller.calculate(inputs.velocityRPM) / FlywheelConstants.maxRPM *
    // Constants.maxVoltage;
    // double feedforward = feedforwardController.calculate(inputs.velocityRPM);
    // appliedVolts = MathUtil.clamp(feedback + feedforward, -12, 12);

    // sim.setInputVoltage(appliedVolts);
    // sim.update(Constants.loopPeriodSecs);

    inputs.appliedVolts = sim.getInputVoltage();

    // TODO: whether it can shoot should be based on whether it has passed through indexer
    // (create Indexer subsystem and IndexerViz)
    shooterSim.update(inputs.velocityRPM, inputs.velocityRPM > 100);
  }

  @Override
  public void runVoltage(double volts) {
    appliedVolts = volts;
  }

  @Override
  public void runVelocity(double velocityRPM) {
    sim.setAngularVelocity(Units.rotationsToRadians(velocityRPM) / 60);
  }
}
