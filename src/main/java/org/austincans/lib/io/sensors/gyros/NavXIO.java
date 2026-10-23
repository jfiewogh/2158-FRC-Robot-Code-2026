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

package org.austincans.lib.io.sensors.gyros;

import com.studica.frc.AHRS;
import com.studica.frc.AHRS.NavXComType;
import com.studica.frc.AHRS.NavXUpdateRate;

import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.AngularVelocity;

import static edu.wpi.first.units.Units.Degrees;
import static edu.wpi.first.units.Units.DegreesPerSecond;

/** IO implementation for NavX. */
public class NavXIO extends AbstractGyroIO {
  private final AHRS navX;

  protected NavXIO(NavXComType comType, NavXUpdateRate updateRate) {
    navX = new AHRS(comType, updateRate);
  }

  @Override
  protected boolean isConnected() {
    return navX.isConnected();
  }

  /* Values are flipped because navX axes are opposite direction of those used by WPILib */
  @Override
  protected Angle getYawPosition() {
    return Degrees.of(-navX.getYaw());
  }
  @Override
  protected AngularVelocity getYawVelocity() {
    return DegreesPerSecond.of(-navX.getRawGyroZ());
  }
  @Override
  protected Angle getPitchPosition() {
    return Degrees.of(-navX.getPitch());
  }
  @Override
  protected AngularVelocity getPitchVelocity() {
    return DegreesPerSecond.of(-navX.getRawGyroX());
  }
  @Override
  protected Angle getRollPosition() {
    return Degrees.of(-navX.getRoll());
  }
  @Override
  protected AngularVelocity getRollVelocity() {
    return DegreesPerSecond.of(-navX.getRawGyroY());
  }
}
