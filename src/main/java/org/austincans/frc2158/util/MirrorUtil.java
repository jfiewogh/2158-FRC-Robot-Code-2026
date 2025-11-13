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
// Updated on Thursday November 13, 2025 at 17:43:29 CST

package org.austincans.frc2158.util;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import java.util.function.BooleanSupplier;
import lombok.Setter;
import lombok.experimental.ExtensionMethod;
import org.austincans.frc2158.FieldConstants;
import org.austincans.frc2158.FieldConstants.CoralObjective;
import org.austincans.frc2158.subsystems.drive.trajectory.TrajectoryGenerationHelpers;
import org.austincans.frc2158.util.vehicletrajectoryservice.VehicleTrajectoryServiceOuterClass.ModuleForce;
import org.austincans.frc2158.util.vehicletrajectoryservice.VehicleTrajectoryServiceOuterClass.VehicleState;

@ExtensionMethod({TrajectoryGenerationHelpers.class})
public class MirrorUtil {
  @Setter private static BooleanSupplier mirror;

  public static CoralObjective apply(CoralObjective coralObjective) {
    if (!shouldMirror()) return coralObjective;
    int shiftedBranchId = coralObjective.branchId() - 1;
    if (shiftedBranchId == -1) {
      shiftedBranchId = 11;
    }
    int flippedBranchId = 11 - shiftedBranchId;
    flippedBranchId = (++flippedBranchId == 12) ? 0 : flippedBranchId;
    return new CoralObjective(flippedBranchId, coralObjective.reefLevel());
  }

  public static double apply(double y) {
    return shouldMirror() ? FieldConstants.fieldWidth - y : y;
  }

  public static Translation2d apply(Translation2d translation) {
    if (!shouldMirror()) return translation;
    return new Translation2d(translation.getX(), apply(translation.getY()));
  }

  public static Pose2d apply(Pose2d pose) {
    if (!shouldMirror()) return pose;
    return new Pose2d(
        apply(pose.getTranslation()),
        new Rotation2d(pose.getRotation().getCos(), -pose.getRotation().getSin()));
  }

  public static VehicleState apply(VehicleState state) {
    if (!shouldMirror()) return state;
    Pose2d pose = apply(state.getPose());
    return VehicleState.newBuilder()
        .setX(pose.getX())
        .setY(pose.getY())
        .setTheta(pose.getRotation().getRadians())
        .setVx(state.getVx())
        .setVy(-state.getVy())
        .setOmega(-state.getOmega())
        .addAllModuleForces(
            state.getModuleForcesList().stream()
                .map(
                    forces ->
                        ModuleForce.newBuilder()
                            .setFx(forces.getFx())
                            .setFy(-forces.getFy())
                            .build())
                .toList())
        .build();
  }

  public static boolean shouldMirror() {
    return mirror != null && mirror.getAsBoolean();
  }
}
