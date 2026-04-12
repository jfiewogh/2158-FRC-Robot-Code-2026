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
import static edu.wpi.first.units.Units.RotationsPerSecond;
import static edu.wpi.first.units.Units.Seconds;
import static edu.wpi.first.units.Units.Volts;

import com.ctre.phoenix6.BaseStatusSignal;
import com.ctre.phoenix6.StatusSignal;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.Follower;
import com.ctre.phoenix6.controls.MotionMagicVelocityTorqueCurrentFOC;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.MotorAlignmentValue;
import com.ctre.phoenix6.signals.NeutralModeValue;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.units.measure.Current;
import edu.wpi.first.units.measure.Temperature;
import edu.wpi.first.units.measure.Voltage;
import java.util.ArrayList;
import java.util.List;
import org.austincans.frc2026.subsystems.selfcheck.AdvancedSubsystem;

public class FlywheelIOTalonFX extends FlywheelIOReal {
  private final TalonFX motor1 = new TalonFX(FlywheelConstants.motor1Id, FlywheelConstants.canBus);
  private final TalonFX motor2 = new TalonFX(FlywheelConstants.motor2Id, FlywheelConstants.canBus);

  private final StatusSignal<AngularVelocity> flywheelVelocity;
  private final StatusSignal<Voltage> flywheelAppliedVolts;
  private final StatusSignal<Current> flywheelCurrent;
  private final StatusSignal<Temperature> flywheelTemp;

  private TalonFXConfiguration talonFXConfigs = new TalonFXConfiguration();

  public FlywheelIOTalonFX() {
    talonFXConfigs.MotorOutput.NeutralMode = NeutralModeValue.Coast;
    talonFXConfigs.MotorOutput.Inverted = InvertedValue.Clockwise_Positive;
    talonFXConfigs.CurrentLimits.StatorCurrentLimit = FlywheelConstants.statorCurrentLimit.in(Amps);
    talonFXConfigs.CurrentLimits.StatorCurrentLimitEnable = true;
    talonFXConfigs.CurrentLimits.SupplyCurrentLimit = FlywheelConstants.supplyCurrentLimit.in(Amps);
    talonFXConfigs.CurrentLimits.SupplyCurrentLimitEnable = true;
    talonFXConfigs.ClosedLoopRamps.withDutyCycleClosedLoopRampPeriod(
        FlywheelConstants.flywheelClosedLoopRamp.in(Seconds));
    talonFXConfigs.OpenLoopRamps.withDutyCycleOpenLoopRampPeriod(
        FlywheelConstants.flywheelOpenLoopRamp.in(Seconds));

    // set slot 0 gains
    var slot0Configs = talonFXConfigs.Slot0;
    slot0Configs.kS = 0; // Static friction
    slot0Configs.kV = 0.01112; // Velocity to Voltage
    slot0Configs.kA = 0; // Acceleration to Voltage
    slot0Configs.kP = 2; // Proportional
    slot0Configs.kI = 0; // Integral
    slot0Configs.kD = 1; // Derivative

    // set Motion Magic Expo settings
    var motionMagicConfigs = talonFXConfigs.MotionMagic;
    motionMagicConfigs.MotionMagicAcceleration = 200; // Unlimited cruise velocity
    motionMagicConfigs.MotionMagicExpo_kV = 0.12; // kV is around 0.12 V/rps
    motionMagicConfigs.MotionMagicExpo_kA = 0.1; // Use a slower kA of 0.1 V/(rps/s)

    motor1.getConfigurator().apply(talonFXConfigs);
    motor2.setControl(new Follower(FlywheelConstants.motor1Id, MotorAlignmentValue.Opposed));
    // talonFXConfigs.MotorOutput.Inverted = InvertedValue.CounterClockwise_Positive;
    // motor2.getConfigurator().apply(talonFXConfigs);

    flywheelVelocity = motor1.getVelocity();
    flywheelAppliedVolts = motor1.getMotorVoltage();
    flywheelCurrent = motor1.getStatorCurrent();
    flywheelTemp = motor1.getDeviceTemp();

    List<BaseStatusSignal> signals = new ArrayList<>();
    signals.add(flywheelVelocity);
    signals.add(flywheelAppliedVolts);
    signals.add(flywheelCurrent);
    signals.add(flywheelTemp);
  }

  @Override
  public void updateInputs(FlywheelIOInputs inputs) {
    List<BaseStatusSignal> signals = new ArrayList<>();
    signals.add(flywheelVelocity);
    signals.add(flywheelAppliedVolts);
    signals.add(flywheelCurrent);
    signals.add(flywheelTemp);

    BaseStatusSignal.refreshAll(signals.toArray(new BaseStatusSignal[0]));

    inputs.velocity = flywheelVelocity.getValue(); // Convert to RPM
    inputs.appliedVolts = flywheelAppliedVolts.getValue();
    inputs.current = flywheelCurrent.getValue();
    inputs.temperature = flywheelTemp.getValue();
  }

  private void runVoltage(Voltage volts) {
    motor1.setVoltage(volts.in(Volts));
  }

  private void runVelocity(AngularVelocity velocity) {
    final MotionMagicVelocityTorqueCurrentFOC m_request =
        new MotionMagicVelocityTorqueCurrentFOC(velocity.in(RotationsPerSecond)).withFeedForward(7);
    motor1.setControl(m_request);
    // motor2.setControl(m_request);
  }

  @Override
  public void applyOutputs(FlywheelIOOutputs outputs) {
    switch (outputs.mode) {
      case VELOCITY -> {
        runVelocity(outputs.velocity);
      }
      case VOLTAGE -> {
        runVoltage(outputs.voltage);
      }
    }
  }

  @Override
  public void registerSelfCheckHardware(AdvancedSubsystem subsystem) {
    subsystem.registerHardware("Flywheel Motor 1", motor1);
    subsystem.registerHardware("Flywheel Motor 2", motor2);
  }
}
