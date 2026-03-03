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

package org.ironmaple.simulation;

import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Translation3d;
import java.util.List;
import org.ironmaple.simulation.physics.PhysicsBody;
import org.ironmaple.simulation.physics.PhysicsEngine;
import org.ironmaple.simulation.physics.PhysicsShape;
import org.littletonrobotics.junction.Logger;

/** This should be robot relative. This is attached to the drive simulation. */
public class MeshSimulation3D implements SimulatedArena3D.Simulatable {
  private final SimulatedArena3D arena;
  private final PhysicsEngine physicsEngine;
  private final PhysicsShape shape;
  private final PhysicsBody body;

  /**
   * SimulatedArena3D must be initialized before this can be created. Origin pose represents another
   * object it is attached to. Currently has same functions has registerWithArena
   */
  public MeshSimulation3D(String meshName, double massKg) {
    arena = SimulatedArena3D.getInstance();
    physicsEngine = arena.getPhysicsEngine();
    shape = physicsEngine.createCompoundShapeFromMesh("meshes/" + meshName);
    body = physicsEngine.createDynamicBody(shape, massKg, Pose3d.kZero);
    arena.addCustomSimulation(this);
  }

  @Override
  public void simulationSubTick(int subTickNum) {
    List<PhysicsBody> overlaps = physicsEngine.getOverlappingBodies(shape, getPose3d());
    Logger.recordOutput("mesh overlaps", overlaps.size());
  }

  public void setPose3d(Pose3d pose) {
    body.setPose3d(pose);
  }

  public void setAngularVelocityRadPerSec(Translation3d angularVelocityRadPerSec) {
    body.setAngularVelocityRadPerSec(angularVelocityRadPerSec);
  }

  public void setRoll(Rotation2d roll) {
    Pose3d currentPose = getPose3d();
    Rotation3d currentRotation = currentPose.getRotation();
    body.setPose3d(
        new Pose3d(
            currentPose.getTranslation(),
            new Rotation3d(roll.getRadians(), currentRotation.getY(), currentRotation.getZ())));
  }

  public void setPitch(Rotation2d pitch) {
    Pose3d currentPose = getPose3d();
    Rotation3d currentRotation = currentPose.getRotation();
    body.setPose3d(
        new Pose3d(
            currentPose.getTranslation(),
            new Rotation3d(currentRotation.getX(), pitch.getRadians(), currentRotation.getZ())));
  }

  public void setYaw(Rotation2d yaw) {
    Pose3d currentPose = getPose3d();
    Rotation3d currentRotation = currentPose.getRotation();
    body.setPose3d(
        new Pose3d(
            currentPose.getTranslation(),
            new Rotation3d(currentRotation.getX(), currentRotation.getY(), yaw.getRadians())));
  }

  public void setAngularPositionRad(Translation3d angularPositionRad) {
    Pose3d currentPose = getPose3d();
    body.setPose3d(
        new Pose3d(
            currentPose.getTranslation(),
            new Rotation3d(
                angularPositionRad.getX(), angularPositionRad.getY(), angularPositionRad.getZ())));
  }

  public Pose3d getPose3d() {
    return body.getPose3d();
  }

  public PhysicsBody getBody() {
    return body;
  }
}
