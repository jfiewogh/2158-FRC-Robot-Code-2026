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
// Updated on Wednesday January 28, 2026 at 19:56:47 CST

package org.austincans.frc2158.util.gslam;

import static edu.wpi.first.units.Units.Celsius;
import static edu.wpi.first.units.Units.Radians;
import static edu.wpi.first.units.Units.RadiansPerSecond;
import static org.austincans.frc2158.util.PhoenixUtil.tryUntilOk;

import com.ctre.phoenix6.BaseStatusSignal;
import com.ctre.phoenix6.CANBus;
import com.ctre.phoenix6.StatusSignal;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.NeutralOut;
import com.ctre.phoenix6.controls.TorqueCurrentFOC;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.units.measure.Current;
import edu.wpi.first.units.measure.Temperature;
import edu.wpi.first.units.measure.Voltage;

public class GenericSlamIOKrakenFOC implements GenericSlamIO {
  // Hardware
  private final TalonFX talon;

  // Status Signals
  private final StatusSignal<Angle> position;
  private final StatusSignal<AngularVelocity> velocity;
  private final StatusSignal<Voltage> appliedVoltage;
  private final StatusSignal<Current> supplyCurrent;
  private final StatusSignal<Current> torqueCurrent;
  private final StatusSignal<Temperature> temp;

  // Control
  private final TorqueCurrentFOC currentControl = new TorqueCurrentFOC(0.0).withUpdateFreqHz(0.0);
  private final NeutralOut neutralOut = new NeutralOut();

  public GenericSlamIOKrakenFOC(
      int id, String bus, int currentLimitAmps, boolean invert, double reduction) {
    talon = new TalonFX(id, new CANBus(bus));

    TalonFXConfiguration config = new TalonFXConfiguration();
    config.MotorOutput.Inverted =
        invert ? InvertedValue.Clockwise_Positive : InvertedValue.CounterClockwise_Positive;
    config.MotorOutput.NeutralMode = NeutralModeValue.Brake;
    config.CurrentLimits.SupplyCurrentLimit = currentLimitAmps;
    config.CurrentLimits.SupplyCurrentLimitEnable = true;
    config.Feedback.SensorToMechanismRatio = reduction;
    tryUntilOk(5, () -> talon.getConfigurator().apply(config));

    position = talon.getPosition();
    velocity = talon.getVelocity();
    appliedVoltage = talon.getMotorVoltage();
    supplyCurrent = talon.getSupplyCurrent();
    torqueCurrent = talon.getTorqueCurrent();
    temp = talon.getDeviceTemp();

    BaseStatusSignal.setUpdateFrequencyForAll(
        50.0, position, velocity, appliedVoltage, supplyCurrent, torqueCurrent, temp);
    talon.optimizeBusUtilization(0, 1.0);
  }

  @Override
  public void updateInputs(GenericSlamIOInputs inputs) {
    inputs.data =
        new GenericSlamIOData(
            BaseStatusSignal.isAllGood(
                position, velocity, appliedVoltage, supplyCurrent, torqueCurrent, temp),
            position.getValue().in(Radians),
            velocity.getValue().in(RadiansPerSecond),
            appliedVoltage.getValueAsDouble(),
            supplyCurrent.getValueAsDouble(),
            torqueCurrent.getValueAsDouble(),
            temp.getValue().in(Celsius));
  }

  @Override
  public void runTorqueCurrent(double amps) {
    talon.setControl(currentControl.withOutput(amps));
  }

  @Override
  public void stop() {
    talon.setControl(neutralOut);
  }

  @Override
  public void setBrakeMode(boolean enable) {
    talon.setNeutralMode(enable ? NeutralModeValue.Brake : NeutralModeValue.Coast);
  }
}
