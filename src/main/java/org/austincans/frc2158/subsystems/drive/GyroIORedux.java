// Copyright (c) 2025 the ausTIN CANs
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
// Updated on Wednesday November 05, 2025 at 17:57:47 CST

package org.austincans.frc2158.subsystems.drive;

import com.reduxrobotics.sensors.canandgyro.Canandgyro;
import com.reduxrobotics.sensors.canandgyro.CanandgyroSettings;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.util.Units;
import java.util.Queue;

public class GyroIORedux implements GyroIO {
  private final Canandgyro gyro = new Canandgyro(30);

  private final Queue<Double> yawTimestampQueue;
  private final Queue<Double> yawPositionQueue;

  public GyroIORedux() {
    // Configure the gyro
    CanandgyroSettings settings =
        new CanandgyroSettings()
            .setYawFramePeriod(1.0 / DriveConstants.odometryFrequency)
            .setAngularPositionFramePeriod(0.01)
            .setAngularVelocityFramePeriod(0.01);
    gyro.setSettings(settings, 0.25, 5);
    gyro.setYaw(0.0, 0.1);
    gyro.clearStickyFaults();

    // Register the gyro signals
    yawTimestampQueue = PhoenixOdometryThread.getInstance().makeTimestampQueue();
    yawPositionQueue = PhoenixOdometryThread.getInstance().registerSignal(gyro::getYaw);
  }

  @Override
  public void updateInputs(GyroIOInputs inputs) {
    inputs.data =
        new GyroIOData(
            gyro.isConnected(),
            Rotation2d.fromRotations(gyro.getYaw()),
            Units.rotationsToRadians(gyro.getAngularVelocityYaw()),
            Rotation2d.fromRotations(gyro.getPitch()),
            Units.rotationsToRadians(gyro.getAngularVelocityPitch()),
            Rotation2d.fromRotations(gyro.getRoll()),
            Units.rotationsToRadians(gyro.getAngularVelocityRoll()));

    inputs.odometryYawTimestamps =
        yawTimestampQueue.stream().mapToDouble((Double value) -> value).toArray();
    inputs.odometryYawPositions =
        yawPositionQueue.stream().map(Rotation2d::fromRotations).toArray(Rotation2d[]::new);

    yawTimestampQueue.clear();
    yawPositionQueue.clear();
  }
}
