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
// Updated on Wednesday November 19, 2025 at 19:28:08 CST

package org.austincans.frc2158.subsystems.intake;

import edu.wpi.first.math.controller.ArmFeedforward;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import org.austincans.frc2158.util.ConfigUtil;
import org.littletonrobotics.junction.AutoLog;

/** Add your docs here. */
public interface IntakeIO {
  public final int deployCanID = ConfigUtil.getConfigInt("deployCanID", 0);
  public final int intakeCanID = ConfigUtil.getConfigInt("intakeCanID", 0);

  /*
   * PID
   * kP: Proportional
   * kI: Integral
   * kD: Derivative
   */
  public double kP = ConfigUtil.getConfigDouble("intakekP", 0);
  public double kI = ConfigUtil.getConfigDouble("intakekI", 0);
  public double kD = ConfigUtil.getConfigDouble("intakekD", 0);

  public TrapezoidProfile.Constraints constraints =
      new TrapezoidProfile.Constraints(
          ConfigUtil.getConfigDouble("intakeMaxVelocityRotations", 0),
          ConfigUtil.getConfigDouble("intakeMaxAcceleration", 0));

  /** Feedforward kS: Static kG: Gravity kV: Velocity kA: Acceleration */
  public double kS = ConfigUtil.getConfigDouble("intakekS", 0);

  public double kG = ConfigUtil.getConfigDouble("intakekG", 0);
  public double kV = ConfigUtil.getConfigDouble("intakekV", 0);
  public double kA = ConfigUtil.getConfigDouble("intakekA", 0);
  public final ArmFeedforward feedforward = new ArmFeedforward(kS, kG, kV, kA);
  public ProfiledPIDController intakePIDController =
      new ProfiledPIDController(kP, kI, kD, constraints);

  public boolean deployBreakEnabled = ConfigUtil.getConfigBoolean("intakeDeployBrakeEnabled", true);

  @AutoLog
  public class IntakeIOInputs {
    public double voltage = 0;
    public double deployPosition = 0;
    public double intakeTemperatureC = 0;
    public double deployTemperatureC = 0;
  }

  public default void updateInputs(IntakeIOInputs inputs) {}

  // rpm
  public default void setRelativeVelocity(double velocity) {}

  public default void setVoltage(double voltage) {}

  // degrees
  public default void setDeployPosition(double degrees) {}
}
