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
// Updated on Sunday February 22, 2026 at 14:08:53 CST

package org.ironmaple.simulation.seasonspecific.reefscape2025;

import static edu.wpi.first.units.Units.*;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.Distance;
import edu.wpi.first.units.measure.LinearVelocity;
import edu.wpi.first.wpilibj.DriverStation;
import org.ironmaple.simulation.Arena;
import org.ironmaple.simulation.gamepieces.GamePieceProjectile;
import org.ironmaple.utils.FieldMirroringUtils;

public class ReefscapeCoralOnFly extends GamePieceProjectile {
  public ReefscapeCoralOnFly(
      Translation2d robotPosition,
      Translation2d shooterPositionOnRobot,
      ChassisSpeeds chassisSpeeds,
      Rotation2d shooterFacing,
      Distance initialHeight,
      LinearVelocity launchingSpeed,
      Angle shooterAngle) {
    super(
        ReefscapeCoralOnField.REEFSCAPE_CORAL_INFO,
        robotPosition,
        shooterPositionOnRobot,
        chassisSpeeds,
        shooterFacing,
        initialHeight,
        launchingSpeed,
        shooterAngle);
    super.enableBecomesGamePieceOnFieldAfterTouchGround();
    super.withTouchGroundHeight(0.2);
  }

  public enum CoralStationsSide {
    LEFT_STATION(new Pose2d(0.89, 7.32, Rotation2d.fromDegrees(-54))),
    RIGHT_STATION(new Pose2d(0.89, 0.6, Rotation2d.fromDegrees(54)));

    private final Pose2d startingPose;

    CoralStationsSide(Pose2d startingPose) {
      this.startingPose = startingPose;
    }
  }

  public static ReefscapeCoralOnFly DropFromCoralStation(
      CoralStationsSide station, DriverStation.Alliance alliance, boolean isHorizontal) {
    Rotation2d rot =
        alliance == DriverStation.Alliance.Red
            ? FieldMirroringUtils.flip(station.startingPose.getRotation())
            : station.startingPose.getRotation();
    Translation2d pos =
        alliance == DriverStation.Alliance.Red
            ? FieldMirroringUtils.flip(station.startingPose.getTranslation())
            : station.startingPose.getTranslation();
    return isHorizontal
        ? new ReefscapeCoralOnFly(
            pos,
            new Translation2d(),
            ChassisSpeeds.fromRobotRelativeSpeeds(new ChassisSpeeds(3.0, 0, 0), rot),
            rot.rotateBy(Rotation2d.kCCW_90deg),
            Centimeters.of(98),
            MetersPerSecond.of(0),
            Degrees.of(0))
        : new ReefscapeCoralOnFly(
            alliance == DriverStation.Alliance.Red
                ? FieldMirroringUtils.flip(station.startingPose.getTranslation())
                : station.startingPose.getTranslation(),
            new Translation2d(),
            new ChassisSpeeds(),
            alliance == DriverStation.Alliance.Red
                ? FieldMirroringUtils.flip(station.startingPose.getRotation())
                : station.startingPose.getRotation(),
            Centimeters.of(98),
            MetersPerSecond.of(3),
            Degrees.of(-50));
  }

  @Override
  public void addGamePieceAfterTouchGround(Arena arena) {
    if (!super.becomesGamePieceOnGroundAfterTouchGround) return;
    arena.spawnGamePieceOnField(
        ReefscapeCoralOnField.REEFSCAPE_CORAL_INFO,
        new Pose3d(
            getPositionAtTime(super.launchedTimer.get()),
            new edu.wpi.first.math.geometry.Rotation3d(
                0, 0, super.initialLaunchingVelocityMPS.getAngle().getRadians())),
        getVelocity3dMPS());
  }
}
