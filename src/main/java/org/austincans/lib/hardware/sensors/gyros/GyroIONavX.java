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

package org.austincans.lib.hardware.sensors.gyros;

import com.studica.frc.AHRS;
import com.studica.frc.AHRS.NavXComType;
import com.studica.frc.AHRS.NavXUpdateRate;

import static edu.wpi.first.units.Units.Degrees;
import static edu.wpi.first.units.Units.DegreesPerSecond;

import org.austincans.lib.subsystems.selfcheck.AdvancedSubsystem;

/** IO implementation for NavX. */
public class GyroIONavX implements GyroIO {
  private final AHRS navX;

  public GyroIONavX(NavXUpdateRate updateRate) {
    navX = new AHRS(NavXComType.kMXP_SPI, updateRate);
  }

  @Override
  public void updateInputs(GyroIOInputs inputs) {
    inputs.setConnected(navX.isConnected());

    // values are flipped because navX axes are opposite direction of those used by WPILib
    inputs.setYawPosition(Degrees.of(-navX.getYaw()));
    inputs.setYawVelocity(DegreesPerSecond.of(-navX.getRawGyroZ()));
    inputs.setPitchPosition(Degrees.of(-navX.getPitch()));
    inputs.setPitchVelocity(DegreesPerSecond.of(-navX.getRawGyroX()));
    inputs.setRollPosition(Degrees.of(-navX.getRoll()));
    inputs.setRollVelocity(DegreesPerSecond.of(-navX.getRawGyroY()));
  }

  @Override
  public void registerSelfCheckHardware(AdvancedSubsystem subsystem) {}
}
