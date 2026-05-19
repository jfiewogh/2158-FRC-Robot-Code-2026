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

package org.austincans.lib.sensors.gyros;

import static edu.wpi.first.units.Units.RadiansPerSecond;
import edu.wpi.first.math.geometry.Rotation2d;

import org.austincans.lib.subsystems.selfcheck.AdvancedSubsystem;
import org.austincans.lib.util.PhoenixUtil;
import org.ironmaple.simulation.drivesims.GyroSimulation;

public class GyroIOSim implements GyroIO {
  private final GyroSimulation gyroSimulation;

  public GyroIOSim(GyroSimulation gyroSimulation) {
    this.gyroSimulation = gyroSimulation;
  }

  @Override
  public void updateRecords(GyroIORecords records) {
    records.connected = true;
    records.yawPosition = gyroSimulation.getGyroReading();
    records.yawVelocityRadPerSec = gyroSimulation.getMeasuredAngularVelocity().in(RadiansPerSecond);
    records.pitchPosition = Rotation2d.kZero;
    records.pitchVelocityRadPerSec = 0.0;
    records.rollPosition = Rotation2d.kZero;
    records.rollVelocityRadPerSec = 0.0;

    records.odometryYawTimestamps = PhoenixUtil.getSimulationOdometryTimeStamps();
    records.odometryYawPositions = gyroSimulation.getCachedGyroReadings();
  }

  @Override
  public void registerSelfCheckHardware(AdvancedSubsystem subsystem) {}
}
