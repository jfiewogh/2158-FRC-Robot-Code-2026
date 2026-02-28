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

package org.austincans.frc2158.subsystems.sim;

import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.math.util.Units;
import java.util.function.Supplier;
import org.austincans.frc2158.subsystems.drive.Drive;
import org.austincans.frc2158.subsystems.turret.TurretConstants;
import org.littletonrobotics.junction.Logger;

// This is specifically for the 2026 robot
public class TurretViz {
  private static final Rotation2d hoodDefaultAngle =
      Rotation2d.fromDegrees(90 - (30.719897 + 8.868396) * 0.5);

  double radius = Units.inchesToMeters(9.747475);

  private final String key;

  // TODO: move to constants
  private final Translation3d[] modelTranslations =
      new Translation3d[] {
        new Translation3d(0, 0, TurretConstants.turretHeightMeters),
        new Translation3d(0, 0, TurretConstants.turretHeightMeters),
        new Translation3d(0.127981, 0, 0.106538 + TurretConstants.turretHeightMeters)
      };

  private final Pose3d[] models = new Pose3d[] {Pose3d.kZero, Pose3d.kZero, Pose3d.kZero};

  private final Supplier<Rotation2d> hoodAngle;

  public TurretViz(String key, Supplier<Rotation2d> hoodAngle, Drive drive) {
    this.key = key;
    this.hoodAngle = hoodAngle;

    models[0] = new Pose3d(modelTranslations[0], Rotation3d.kZero);
  }

  public void update(Rotation2d turretAngle) {
    double yawRad = turretAngle.getRadians();

    models[1] = new Pose3d(modelTranslations[1], new Rotation3d(0, 0, yawRad));

    double pitchRad =
        Math.PI / 2 - hoodAngle.get().getRadians() - (Math.PI / 2 - hoodDefaultAngle.getRadians());

    models[2] =
        new Pose3d(
            new Translation3d(modelTranslations[2].getX(), modelTranslations[2].getY(), 0)
                .rotateBy(new Rotation3d(0, 0, yawRad))
                .plus(new Translation3d(0, 0, modelTranslations[2].getZ())),
            new Rotation3d(0, pitchRad, yawRad));

    for (int i = 0; i < models.length; i++) {
      Logger.recordOutput(key + "/Turret" + i, models[i]);
    }
  }
}
