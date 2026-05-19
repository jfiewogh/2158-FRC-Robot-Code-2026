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

import static edu.wpi.first.units.Units.Hertz;
import static edu.wpi.first.units.Units.Radians;
import static edu.wpi.first.units.Units.RadiansPerSecond;
import com.ctre.phoenix6.BaseStatusSignal;
import com.ctre.phoenix6.CANBus;
import com.ctre.phoenix6.StatusSignal;
import com.ctre.phoenix6.configs.Pigeon2Configuration;
import com.ctre.phoenix6.hardware.Pigeon2;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.AngularVelocity;
import java.util.Queue;
import org.austincans.frc2026.generated.TunerConstants;
import org.austincans.lib.subsystems.selfcheck.AdvancedSubsystem;
import org.austincans.lib.util.PhoenixUtil;

/** IO implementation for Pigeon 2. */
public class GyroIOPigeon2 implements GyroIO {
  private final Pigeon2 pigeon =
      new Pigeon2(
          TunerConstants.DrivetrainConstants.Pigeon2Id,
          new CANBus(TunerConstants.DrivetrainConstants.CANBusName));

  private final StatusSignal<Angle> yaw;
  private final StatusSignal<Angle> pitch;
  private final StatusSignal<Angle> roll;
  private final Queue<Double> yawPositionQueue;
  private final Queue<Double> yawTimestampQueue;
  private final StatusSignal<AngularVelocity> yawVelocity;
  private final StatusSignal<AngularVelocity> pitchVelocity;
  private final StatusSignal<AngularVelocity> rollVelocity;

  public GyroIOPigeon2(GyroConfigPigeon2 gyroConfig, Pigeon2Configuration config) {
    pigeon = new Pigeon2(gyroConfig.deviceId, new CANBus(gyroConfig.canbus));
    pigeon.getConfigurator().apply(config);
    pigeon.getConfigurator().setYaw(0.0);

    yaw = pigeon.getYaw();
    pitch = pigeon.getPitch();
    roll = pigeon.getRoll();
    yawVelocity = pigeon.getAngularVelocityZWorld();
    pitchVelocity = pigeon.getAngularVelocityXWorld();
    rollVelocity = pigeon.getAngularVelocityYWorld();

    yaw.setUpdateFrequency(Frequency.ofBaseUnits(gyroConfig.updateFrequencyHz, Hertz));
    BaseStatusSignal.setUpdateFrequencyForAll(
        50, pitch, roll, yawVelocity, pitchVelocity, rollVelocity);
    pigeon.optimizeBusUtilization();

    yawTimestampQueue = PhoenixOdometryThread.getInstance().makeTimestampQueue();
    yawPositionQueue = PhoenixOdometryThread.getInstance().registerSignal(yaw);

    PhoenixUtil.registerSignals(true, yaw, yawVelocity, pitch, pitchVelocity, roll, rollVelocity);
    PhoenixUtil.tryUntilOk(5, () -> pigeon.setYaw(0.0, 0.25));
  }

  @Override
  public void updateRecords(GyroIORecords records) {
    records.connected =
        BaseStatusSignal.isAllGood(yaw, yawVelocity, pitch, pitchVelocity, roll, rollVelocity);

    records.yawPosition = Rotation2d.fromRadians(yaw.getValue().in(Radians));
    records.yawVelocityRadPerSec = yawVelocity.getValue().in(RadiansPerSecond);
    records.pitchPosition = Rotation2d.fromRadians(pitch.getValue().in(Radians));
    records.pitchVelocityRadPerSec = pitchVelocity.getValue().in(RadiansPerSecond);
    records.rollPosition = Rotation2d.fromRadians(roll.getValue().in(Radians));
    records.rollVelocityRadPerSec = rollVelocity.getValue().in(RadiansPerSecond);

    yawTimestampQueue.clear();
    yawPositionQueue.clear();
  }

  @Override
  public void registerSelfCheckHardware(AdvancedSubsystem subsystem) {
    subsystem.registerHardware("Pigeon", pigeon);
  }
}
