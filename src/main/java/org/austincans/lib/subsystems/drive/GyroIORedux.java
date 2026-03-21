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

package org.austincans.lib.subsystems.drive;

import com.reduxrobotics.sensors.canandgyro.Canandgyro;
import com.reduxrobotics.sensors.canandgyro.CanandgyroSettings;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.util.Units;
import java.util.Queue;
import org.austincans.lib.subsystems.drive.old.PhoenixOdometryThread;
import org.austincans.lib.subsystems.selfcheck.AdvancedSubsystem;

public class GyroIORedux implements GyroIO {
  private final Canandgyro gyro = new Canandgyro(30);

  private final Queue<Double> yawTimestampQueue;
  private final Queue<Double> yawPositionQueue;

  public GyroIORedux(double odometryFrequencyHz) {
    // Configure the gyro
    CanandgyroSettings settings =
        new CanandgyroSettings()
            .setYawFramePeriod(1.0 / odometryFrequencyHz)
            .setAngularPositionFramePeriod(0.01)
            .setAngularVelocityFramePeriod(0.01);
    gyro.setSettings(settings, 0.25, 5);
    gyro.setYaw(0.0); // gyro.setYaw(0.0, 0.1);
    gyro.clearStickyFaults();

    // Register the gyro signals
    yawTimestampQueue = PhoenixOdometryThread.getInstance().makeTimestampQueue();
    yawPositionQueue = PhoenixOdometryThread.getInstance().registerSignal(gyro::getYaw);
  }

  @Override
  public void updateRecords(GyroIORecords records) {
    records.connected = gyro.isConnected();
    records.yawPosition = Rotation2d.fromRotations(gyro.getYaw());
    records.yawVelocityRadPerSec = Units.rotationsToRadians(gyro.getAngularVelocityYaw());
    records.pitchPosition = Rotation2d.fromRotations(gyro.getPitch());
    records.pitchVelocityRadPerSec = Units.rotationsToRadians(gyro.getAngularVelocityPitch());
    records.rollPosition = Rotation2d.fromRotations(gyro.getRoll());
    records.rollVelocityRadPerSec = Units.rotationsToRadians(gyro.getAngularVelocityRoll());

    records.odometryYawTimestamps =
        yawTimestampQueue.stream().mapToDouble((Double value) -> value).toArray();
    records.odometryYawPositions =
        yawPositionQueue.stream().map(Rotation2d::fromRotations).toArray(Rotation2d[]::new);

    yawTimestampQueue.clear();
    yawPositionQueue.clear();
  }

  // TODO: add self checking
  @Override
  public void registerSelfCheckHardware(AdvancedSubsystem subsystem) {}
}
