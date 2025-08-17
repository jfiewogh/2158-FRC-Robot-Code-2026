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
// Updated on Sunday August 17, 2025 at 08:59:14 CDT

package org.austincans.frc2158.subsystems.drive;

import static org.austincans.frc2158.util.PhoenixUtil.tryUntilOk;

import com.ctre.phoenix6.BaseStatusSignal;
import com.ctre.phoenix6.StatusSignal;
import com.ctre.phoenix6.configs.Pigeon2Configuration;
import com.ctre.phoenix6.hardware.Pigeon2;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.AngularVelocity;
import java.util.Queue;
import org.austincans.frc2158.subsystems.drive.DriveConstants.PigeonConstants;
import org.austincans.frc2158.util.PhoenixUtil;

/** IO implementation for Pigeon 2. */
public class GyroIOPigeon2 implements GyroIO {
  private final Pigeon2 pigeon = new Pigeon2(PigeonConstants.id, "*");
  private final StatusSignal<Angle> yaw = pigeon.getYaw();
  private final StatusSignal<Angle> pitch = pigeon.getPitch();
  private final StatusSignal<Angle> roll = pigeon.getRoll();
  private final Queue<Double> yawPositionQueue;
  private final Queue<Double> yawTimestampQueue;
  private final StatusSignal<AngularVelocity> yawVelocity = pigeon.getAngularVelocityZWorld();
  private final StatusSignal<AngularVelocity> pitchVelocity = pigeon.getAngularVelocityXWorld();
  private final StatusSignal<AngularVelocity> rollVelocity = pigeon.getAngularVelocityYWorld();

  public GyroIOPigeon2() {
    pigeon.getConfigurator().apply(new Pigeon2Configuration());
    pigeon.getConfigurator().setYaw(0.0);
    yaw.setUpdateFrequency(DriveConstants.odometryFrequency);
    BaseStatusSignal.setUpdateFrequencyForAll(
        50, pitch, roll, yawVelocity, pitchVelocity, rollVelocity);
    pigeon.optimizeBusUtilization();
    yawTimestampQueue = PhoenixOdometryThread.getInstance().makeTimestampQueue();
    yawPositionQueue = PhoenixOdometryThread.getInstance().registerSignal(pigeon.getYaw());
    PhoenixUtil.registerSignals(true, yaw, yawVelocity, pitch, pitchVelocity, roll, rollVelocity);
    tryUntilOk(5, () -> pigeon.setYaw(0.0, 0.25));
  }

  @Override
  public void updateInputs(GyroIOInputs inputs) {
    inputs.data =
        new GyroIOData(
            BaseStatusSignal.isAllGood(yaw, yawVelocity, pitch, pitchVelocity, roll, rollVelocity),
            Rotation2d.fromDegrees(yaw.getValueAsDouble()),
            Units.degreesToRadians(yawVelocity.getValueAsDouble()),
            Rotation2d.fromDegrees(pitch.getValueAsDouble()),
            Units.degreesToRadians(pitchVelocity.getValueAsDouble()),
            Rotation2d.fromDegrees(roll.getValueAsDouble()),
            Units.degreesToRadians(rollVelocity.getValueAsDouble()));

    inputs.odometryYawTimestamps =
        yawTimestampQueue.stream().mapToDouble((Double value) -> value).toArray();
    inputs.odometryYawPositions =
        yawPositionQueue.stream().map(Rotation2d::fromDegrees).toArray(Rotation2d[]::new);
    yawTimestampQueue.clear();
    yawPositionQueue.clear();
  }
}
