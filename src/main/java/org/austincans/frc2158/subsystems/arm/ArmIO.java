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

package org.austincans.frc2158.subsystems.arm;

import edu.wpi.first.math.controller.ArmFeedforward;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import java.util.List;
import org.austincans.frc2158.subsystems.selfcheck.SubsystemFault;
import org.littletonrobotics.junction.AutoLog;

public interface ArmIO {
  public TrapezoidProfile.Constraints constraints =
      new TrapezoidProfile.Constraints(
          ArmConstants.armMaxVelocityRotations, ArmConstants.armMaxAcceleration);

  /** Feedforward kS: Static kG: Gravity kV: Velocity kA: Acceleration */
  public final ArmFeedforward feedforward =
      new ArmFeedforward(ArmConstants.kS, ArmConstants.kG, ArmConstants.kV, ArmConstants.kA);

  public ProfiledPIDController armPIDController =
      new ProfiledPIDController(ArmConstants.kP, ArmConstants.kI, ArmConstants.kD, constraints);

  @AutoLog
  class ArmIOInputs {
    public double velocity = 0;
    public double position = 0;
    public double currentAmps = 0;
    public double currentVoltage = 0;
    public double temperature = 0;
  }

  public void updateInputs(ArmIOInputs inputs);

  public void setVoltage(double voltage)
  ;

  // Encoder Values
  public void setPosition(double position)
  ;

  // Meters per second
  public void setVelocity(double velocity)
  ;

  public default void stopMotor() {}
  ;

  public default List<SubsystemFault> checkForFaults() {
    return null;
  }
}
