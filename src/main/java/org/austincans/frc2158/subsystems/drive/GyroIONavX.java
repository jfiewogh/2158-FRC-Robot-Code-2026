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
//
// Updated on Saturday January 31, 2026 at 12:05:30 CST

package org.austincans.frc2158.subsystems.drive;

import com.studica.frc.AHRS;
import com.studica.frc.AHRS.NavXComType;
import com.studica.frc.AHRS.NavXUpdateRate;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.util.Units;
import java.util.List;
import java.util.Queue;
import org.austincans.frc2158.subsystems.selfcheck.SubsystemFault;

/** IO implementation for NavX. */
public class GyroIONavX implements GyroIO {
  private final AHRS navX =
      new AHRS(
          NavXComType.kMXP_SPI,
          NavXUpdateRate.k100Hz // TODO: should be based on Drive.ODOMETRY_FREQUENCY)
          );
  private final Queue<Double> yawPositionQueue;
  private final Queue<Double> yawTimestampQueue;

  public GyroIONavX() {
    yawTimestampQueue = PhoenixOdometryThread.getInstance().makeTimestampQueue();
    yawPositionQueue = PhoenixOdometryThread.getInstance().registerSignal(navX::getYaw);
  }

  @Override
  public void updateInputs(GyroIOInputs inputs) {
    inputs.connected = navX.isConnected();

    // values are flipped because navX axes are opposite direction of those used by WPILib
    inputs.yawPosition = Rotation2d.fromDegrees(-navX.getYaw());
    inputs.yawVelocityRadPerSec = Units.degreesToRadians(-navX.getRawGyroZ());
    inputs.pitchPosition = Rotation2d.fromDegrees(-navX.getPitch());
    inputs.pitchVelocityRadPerSec = Units.degreesToRadians(-navX.getRawGyroX());
    inputs.rollPosition = Rotation2d.fromDegrees(-navX.getRoll());
    inputs.rollVelocityRadPerSec = Units.degreesToRadians(-navX.getRawGyroY());

    inputs.odometryYawTimestamps =
        yawTimestampQueue.stream().mapToDouble((Double value) -> value).toArray();
    inputs.odometryYawPositions =
        yawPositionQueue.stream()
            .map((Double value) -> Rotation2d.fromDegrees(-value))
            .toArray(Rotation2d[]::new);
    yawTimestampQueue.clear();
    yawPositionQueue.clear();
    inputs.imuRate = navX.getRate();
  }

  // TODO: add check for faults
  @Override
  public List<SubsystemFault> checkForFaults() {
    return List.of();
  }
}
