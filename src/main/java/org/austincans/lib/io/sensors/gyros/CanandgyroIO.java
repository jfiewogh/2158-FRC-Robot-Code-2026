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

import com.reduxrobotics.sensors.canandgyro.Canandgyro;
import com.reduxrobotics.sensors.canandgyro.CanandgyroSettings;

import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.units.measure.Frequency;

import static edu.wpi.first.units.Units.Hertz;
import static edu.wpi.first.units.Units.Rotations;
import static edu.wpi.first.units.Units.RotationsPerSecond;

public class CanandgyroIO extends AbstractGyroIO {
  private final Canandgyro gyro;

  protected CanandgyroIO(int deviceId, Frequency odometryFrequency) {
    gyro = new Canandgyro(deviceId);
    // Configure the gyro
    CanandgyroSettings settings =
        new CanandgyroSettings()
            .setYawFramePeriod(1.0 / odometryFrequency.in(Hertz))
            .setAngularPositionFramePeriod(0.01)
            .setAngularVelocityFramePeriod(0.01);
    gyro.setSettings(settings, 0.25, 5);
    gyro.setYaw(0.0);
    gyro.clearStickyFaults();
  }

  @Override
  protected boolean isConnected() {
    return gyro.isConnected();
  }
  @Override
  protected Angle getYawPosition() {
    return Rotations.of(gyro.getYaw());
  }
  @Override
  protected AngularVelocity getYawVelocity() {
    return RotationsPerSecond.of(gyro.getAngularVelocityYaw());
  }
  @Override
  protected Angle getPitchPosition() {
    return Rotations.of(gyro.getPitch());
  }
  @Override
  protected AngularVelocity getPitchVelocity() {
    return RotationsPerSecond.of(gyro.getAngularVelocityPitch());
  }
  @Override
  protected Angle getRollPosition() {
    return Rotations.of(gyro.getRoll());
  }
  @Override
  protected AngularVelocity getRollVelocity() {
    return RotationsPerSecond.of(gyro.getAngularVelocityRoll());
  }
}
