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

import com.revrobotics.PersistMode;
import com.revrobotics.ResetMode;
import com.revrobotics.spark.ClosedLoopSlot;
import com.revrobotics.spark.SparkBase.ControlType;
import com.revrobotics.spark.SparkClosedLoopController;
import com.revrobotics.spark.SparkFlex;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.config.SparkFlexConfig;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.units.measure.Voltage;
import static edu.wpi.first.units.Units.Amps;
import static edu.wpi.first.units.Units.Celsius;
import static edu.wpi.first.units.Units.RPM;
import static edu.wpi.first.units.Units.Volts;
import org.austincans.frc2026.subsystems.selfcheck.AdvancedSubsystem;

public class FlywheelIOSparkFlex extends FlywheelIOReal {
  private final SparkFlex motor1 = new SparkFlex(FlywheelConstants.motor1Id, MotorType.kBrushless);
  private final SparkFlex motor2 = new SparkFlex(FlywheelConstants.motor2Id, MotorType.kBrushless);

  private final SparkClosedLoopController controller = motor1.getClosedLoopController();
  private final SparkFlexConfig config = new SparkFlexConfig();
  private final SparkFlexConfig config2 = new SparkFlexConfig();

  public FlywheelIOSparkFlex() {
    config.closedLoop.pid(0.00125, 0, 0.15, ClosedLoopSlot.kSlot0);
    config.closedLoop.feedForward.kV(0.001);
    config.encoder.velocityConversionFactor(2);
    config.encoder.positionConversionFactor(2);
    motor1.configureAsync(config, ResetMode.kResetSafeParameters, PersistMode.kNoPersistParameters);
    config2.follow(motor1, true);
    // config.inverted(true);
    motor2.configureAsync(
        config2, ResetMode.kResetSafeParameters, PersistMode.kNoPersistParameters);
  }

  @Override
  public void updateInputs(FlywheelIOInputs inputs) {
    inputs.appliedVolts = Volts.of(motor1.getBusVoltage());
    inputs.velocity = RPM.of(motor1.getEncoder().getVelocity());
    inputs.temperature = Celsius.of(motor1.getMotorTemperature());
    inputs.current = Amps.of(motor1.getOutputCurrent());
  }

  private void runVoltage(Voltage volts) {
    motor1.setVoltage(volts);
  }

  private void runVelocity(AngularVelocity velocity) {
    controller.setSetpoint(velocity.in(RPM) * 2, ControlType.kVelocity, ClosedLoopSlot.kSlot0);
    if (velocity == RPM.of(0)) {
      controller.setSetpoint(0, ControlType.kDutyCycle);
    }
  }

  @Override
  public void applyOutputs(FlywheelIOOutputs outputs) {
    switch (outputs.mode){
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
