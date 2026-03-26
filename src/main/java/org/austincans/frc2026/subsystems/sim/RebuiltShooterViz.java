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

package org.austincans.frc2026.subsystems.sim;

import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.math.util.Units;
import java.util.function.Supplier;
import org.austincans.frc2026.commands.shooter.ShotConstants;
import org.austincans.frc2026.subsystems.drive.Drive;
import org.austincans.frc2026.subsystems.turret.TurretConstants;
import org.littletonrobotics.junction.Logger;

// This is specifically for the 2026 robot
public class RebuiltShooterViz {
  private static final Rotation2d hoodDefaultAngle =
      Rotation2d.fromDegrees(90 - (30.719897 + 8.868396) * 0.5);

  double radius = Units.inchesToMeters(9.747475);

  private final String key;

  public static final Translation3d hoodOffset = new Translation3d(0.127981, 0, 0.106538);

  private final Translation3d shooterBaseTranslation;
  private final Translation3d[] modelTranslations;

  private final Pose3d[] models = new Pose3d[] {Pose3d.kZero, Pose3d.kZero, Pose3d.kZero};

  private final Supplier<Rotation2d> hoodAngle;

  public RebuiltShooterViz(String key, Supplier<Rotation2d> hoodAngle, Drive drive) {
    this.key = key;
    this.hoodAngle = hoodAngle;

    shooterBaseTranslation =
        new Translation3d(
                ShotConstants.robotRelTurretTransform
                    .getTranslation()
                    .toTranslation2d()
                    .rotateBy(ShotConstants.robotRelTurretTransform.getRotation().toRotation2d()))
            .plus(new Translation3d(0, 0, TurretConstants.turretHeightMeters));
    modelTranslations =
        new Translation3d[] {
          shooterBaseTranslation, shooterBaseTranslation, shooterBaseTranslation.plus(hoodOffset)
        };

    models[0] = new Pose3d(modelTranslations[0], Rotation3d.kZero);
  }

  public void update(Rotation2d turretAngle) {
    models[1] =
        new Pose3d(
            modelTranslations[1],
            new Rotation3d(
                turretAngle.plus(
                    ShotConstants.robotRelTurretTransform.getRotation().toRotation2d())));

    models[2] =
        new Pose3d(
            new Translation3d(hoodOffset.getX(), hoodOffset.getY(), 0)
                .rotateBy(new Rotation3d(turretAngle))
                .plus(shooterBaseTranslation.plus(new Translation3d(0, 0, hoodOffset.getZ()))),
            new Rotation3d(0, getPitchRad(hoodAngle.get().getRadians()), turretAngle.getRadians()));

    for (int i = 0; i < models.length; i++) {
      Logger.recordOutput(key + "/Turret" + i, models[i]);
    }
  }

  public static double getPitchRad(double hoodAngleRad) {
    return Math.PI / 2 - hoodAngleRad - (Math.PI / 2 - hoodDefaultAngle.getRadians());
  }
}
