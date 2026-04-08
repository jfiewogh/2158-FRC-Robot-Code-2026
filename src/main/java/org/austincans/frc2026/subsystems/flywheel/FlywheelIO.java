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

import static edu.wpi.first.units.Units.Amps;
import static edu.wpi.first.units.Units.Fahrenheit;
import static edu.wpi.first.units.Units.Rotations;
import static edu.wpi.first.units.Units.RotationsPerSecond;
import static edu.wpi.first.units.Units.Volts;

import java.util.List;

import org.austincans.frc2026.subsystems.selfcheck.AdvancedSubsystem;
import org.austincans.frc2026.subsystems.selfcheck.HardwareFault;
import org.austincans.frc2026.subsystems.selfcheck.SelfChecking;
import org.littletonrobotics.junction.AutoLog;

import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.units.measure.Current;
import edu.wpi.first.units.measure.Temperature;
import edu.wpi.first.units.measure.Velocity;
import edu.wpi.first.units.measure.Voltage;

public interface FlywheelIO {
  @AutoLog
  public class FlywheelIOInputs {
    public Voltage appliedVolts = Volts.of(0);
    public AngularVelocity velocityRPM = RotationsPerSecond.of(0);
    public Temperature temperature = Fahrenheit.of(0);
    public Current current = Amps.of(0);
  }

  public class FlywheelIOOutputs {
    public FlywheelIOOutputsMode mode = FlywheelIOOutputsMode.VELOCITY;
    public Voltage voltage = Volts.of(0);
    public AngularVelocity velocity = RotationsPerSecond.of(0);
    public double feedforward = 0;
  }

  public static enum FlywheelIOOutputsMode {
    VELOCITY,
    VOLTAGE
  }
  public void updateInputs(FlywheelIOInputs inputs);

  public void applyOutputs(FlywheelIOOutputs outputs);

  public void registerSelfCheckHardware(AdvancedSubsystem subsystem);



  public default void runVoltage(Voltage volts) {}

  public default void runVelocity(AngularVelocity velocityRPM) {}

  public default List<HardwareFault> checkForFaults() {
    return List.of();
  }

  public default List<SelfChecking> getHardware() {
    return List.of();
  }

  public default boolean readyToShoot() {
    return false;
  }

  public default boolean isShooting() {
    return false;
  }

  public default boolean noGamePiecesToShoot() {
    return false;
  }

  
}
