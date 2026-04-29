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

package org.austincans.lib.subsystems.pivot;

import static edu.wpi.first.units.Units.Rotations;
import org.austincans.lib.util.KalmanFilter1D;
import org.austincans.lib.encoders.AbsoluteEncoderFactory;
import org.austincans.lib.encoders.CANcoderIO;
import org.austincans.lib.motors.MotorFactory;
import org.austincans.lib.motors.MotorIOSparkFlex;
import org.austincans.lib.subsystems.selfcheck.AdvancedSubsystem;

public class PivotIOSparkFlexCANcoder extends PivotIOReal<MotorIOSparkFlex, CANcoderIO> {
  @SuppressWarnings("unused")
  private KalmanFilter1D filter1d = new KalmanFilter1D(0, 0, 0);

  public PivotIOSparkFlexCANcoder(PivotConfig pivotConfig) {
    super(
      MotorFactory.createSparkFlex(pivotConfig.motorSparkConfig,pivotConfig.sparkFlexConfig), 
      AbsoluteEncoderFactory.createCANcoder(pivotConfig.absoluteEncoderConfig));

    motorIO.setEncoderPosition(absoluteEncoderIO.getAbsolutePosition().in(Rotations));
  }

  @Override
  public void updateInputs(PivotIOInputs inputs) {
    super.updateInputs(inputs);
  }

  @Override
  public void registerSelfCheckHardware(AdvancedSubsystem subsystem) {
    subsystem.registerHardware("pivot", motorIO.getMotor());
  }

  @Override
  public void stop() {
    motorIO.getMotor().set(0);
  }
}
