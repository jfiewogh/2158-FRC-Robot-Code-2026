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
// Updated on Sunday February 22, 2026 at 14:08:53 CST

package org.austincans.frc2158.subsystems.arm;

import edu.wpi.first.math.controller.ArmFeedforward;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import java.util.List;
import org.austincans.frc2158.subsystems.selfcheck.SelfChecking;
import org.austincans.frc2158.subsystems.selfcheck.SubsystemFault;
import org.austincans.frc2158.util.ConfigUtil;
import org.littletonrobotics.junction.AutoLog;

public interface ArmIO {
  /*
   * PID
   * kP: Proportional
   * kI: Integral
   * kD: Derivative
   */
  public double kP = ConfigUtil.getConfigDouble("armkP", 0);
  public double kI = ConfigUtil.getConfigDouble("armkI", 0);
  public double kD = ConfigUtil.getConfigDouble("armkD", 0);

  public TrapezoidProfile.Constraints constraints =
      new TrapezoidProfile.Constraints(
          ConfigUtil.getConfigDouble("armMaxVelocityRotations", 0),
          ConfigUtil.getConfigDouble("armMaxAcceleration", 0));

  /** Feedforward kS: Static kG: Gravity kV: Velocity kA: Acceleration */
  public double kS = ConfigUtil.getConfigDouble("armkS", 0);

  public double kG = ConfigUtil.getConfigDouble("armkG", 0);
  public double kV = ConfigUtil.getConfigDouble("armkV", 0);
  public double kA = ConfigUtil.getConfigDouble("armkA", 0);
  public final ArmFeedforward feedforward = new ArmFeedforward(kS, kG, kV, kA);
  public ProfiledPIDController armPIDController =
      new ProfiledPIDController(kP, kI, kD, constraints);

  @AutoLog
  class ArmIOInputs {
    public double velocity = 0;
    public double position = 0;
    public double currentAmps = 0;
    public double currentVoltage = 0;
    public double temperature = 0;
  }

  public void updateInputs(ArmIOInputs inputs);

  public default void setVoltage(double voltage) {}
  ;

  // Encoder Values
  public default void setPosition(double position) {}
  ;

  // Meters per second
  public default void setVelocity(double velocity) {}
  ;

  public default void stopMotor() {}
  ;

  public default List<SubsystemFault> checkForFaults() {
    return List.of();
  }

  public default List<SelfChecking> getHardware() {
    return List.of();
  }
}
