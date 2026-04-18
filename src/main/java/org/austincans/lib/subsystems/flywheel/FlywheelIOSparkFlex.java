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

package org.austincans.lib.subsystems.flywheel;

import edu.wpi.first.math.util.Units;
import java.util.ArrayList;
import java.util.List;
import org.austincans.lib.motors.MotorFactory;
import org.austincans.lib.motors.MotorIOSparkFlex;
import org.austincans.lib.subsystems.selfcheck.AdvancedSubsystem;
import org.austincans.lib.subsystems.selfcheck.HardwareFault;
import org.austincans.lib.subsystems.selfcheck.SelfChecking;
import org.austincans.lib.subsystems.selfcheck.SelfCheckingSparkFlex;

public class FlywheelIOSparkFlex implements FlywheelIO{
  private final MotorIOSparkFlex motorIO;
  private final SelfCheckingSparkFlex selfCheckMotor;

  public FlywheelIOSparkFlex(FlywheelConfig flywheelConfig) {
    motorIO = MotorFactory.createSparkFlex(flywheelConfig.motorConfig);
    selfCheckMotor = new SelfCheckingSparkFlex("flywheel", motorIO.getMotor());
  }

  @Override
  public void updateInputs(FlywheelIOInputs inputs) {
    motorIO.updateInputs(inputs.motorInputs);
    inputs.velocityRPM = Units.radiansPerSecondToRotationsPerMinute(inputs.motorInputs.velocityRadPerSec);
    inputs.appliedVolts = inputs.motorInputs.appliedVolts;
  }

  @Override
  public void runVoltage(double volts) {
    motorIO.setVoltage(volts);
  }

  @Override
  public void runVelocity(double velocityRPM) {
    motorIO.setVelocity(Units.rotationsPerMinuteToRadiansPerSecond(velocityRPM));
  }

  @Override
  public void applyOutputs(FlywheelIOOutputs outputs) {
    switch (outputs.mode){
      case VELOCITY:
        motorIO.setVelocity(Units.rotationsPerMinuteToRadiansPerSecond(outputs.velocity));
        break;
      case VOLTAGE:
        motorIO.setVoltage(outputs.voltage);
        break;
    }
  }

  @Override
  public void registerSelfCheckHardware(AdvancedSubsystem subsystem) {
    // TODO Auto-generated method stub
    throw new UnsupportedOperationException("Unimplemented method 'registerSelfCheckHardware'");
  }
}
