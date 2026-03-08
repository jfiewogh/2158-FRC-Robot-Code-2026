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

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.math.system.plant.LinearSystemId;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.simulation.FlywheelSim;
import java.util.function.Supplier;
import org.austincans.frc2026.Constants;
import org.austincans.frc2026.subsystems.drive.Drive;
import org.austincans.frc2026.subsystems.sim.ShooterSim;
import org.ironmaple.simulation.IntakeSimulation;

public class FlywheelIOSim implements FlywheelIO {
  private static final boolean isFeedbackControl = false;

  private final ShooterSim shooterSim;
  private final IntakeSimulation intakeSimulation;

  private final DCMotor plant = DCMotor.getNeoVortex(1);
  private final FlywheelSim sim =
      new FlywheelSim(
          LinearSystemId.createFlywheelSystem(
              plant, FlywheelConstants.JKgMetersSq, FlywheelConstants.gearing),
          plant);

  private final PIDController controller = new PIDController(5, 0, 0);
  private final SimpleMotorFeedforward feedforwardController =
      new SimpleMotorFeedforward(FlywheelConstants.kS, FlywheelConstants.kV, FlywheelConstants.kA);

  private double velocityRPM;
  private double appliedVolts;

  /** I am passing in the drive subsystem as a parameter but only using its getter methods */
  public FlywheelIOSim(
      Drive drive,
      IntakeSimulation intakeSimulation,
      Supplier<Pose3d> robotFieldRelPoseSupplier,
      Supplier<Rotation2d> hoodAngleSupplier,
      Supplier<Rotation2d> turretAngle) {
    shooterSim =
        new ShooterSim(
            drive,
            FlywheelConstants.radiusMeters,
            0.1,
            intakeSimulation,
            hoodAngleSupplier,
            turretAngle);
    this.intakeSimulation = intakeSimulation;
  }

  @Override
  public void updateInputs(FlywheelIOInputs inputs) {
    inputs.velocityRPM = sim.getAngularVelocityRPM();
    velocityRPM = inputs.velocityRPM;

    if (isFeedbackControl) {
      double feedback =
          controller.calculate(inputs.velocityRPM)
              / FlywheelConstants.maxRPM
              * Constants.maxVoltage;
      double feedforward = feedforwardController.calculate(inputs.velocityRPM);
      appliedVolts = MathUtil.clamp(feedback + feedforward, -12, 12);

      sim.setInputVoltage(appliedVolts);
      sim.update(Constants.loopPeriodSecs);
    }

    inputs.appliedVolts = sim.getInputVoltage();

    shooterSim.update(inputs.velocityRPM, inputs.velocityRPM > 100);
  }

  @Override
  public void runVoltage(double volts) {
    appliedVolts = volts;
  }

  @Override
  public void runRelativeVelocity(double velocityRPM) {
    sim.setAngularVelocity(Units.rotationsToRadians(velocityRPM) / 60);
  }

  @Override
  public boolean readyToShoot() {
    return intakeSimulation.getGamePiecesAmount() > 0;
  }

  @Override
  public boolean isShooting() {
    return velocityRPM > 100 && readyToShoot();
  }

  @Override
  public boolean noGamePiecesToShoot() {
    return intakeSimulation.getGamePiecesAmount() == 0;
  }
}
