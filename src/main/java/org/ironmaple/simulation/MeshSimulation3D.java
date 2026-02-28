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
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Translation3d;
import java.util.function.Supplier;
import org.ironmaple.simulation.physics.PhysicsBody;
import org.ironmaple.simulation.physics.PhysicsEngine;
import org.ironmaple.simulation.physics.PhysicsShape;

/** This should be robot relative. This is attached to the drive simulation. */
public class MeshSimulation3D implements SimulatedArena3D.Simulatable {
  private final SimulatedArena3D arena;
  private final PhysicsEngine physicsEngine;
  private final PhysicsShape shape;
  private final PhysicsBody body;
  private final Supplier<Pose3d> originPoseSupplier;
  private final Supplier<Translation3d> speedsSupplier;

  /**
   * SimulatedArena3D must be initialized before this can be created. Origin pose represents another
   * object it is attached to. Currently has same functions has registerWithArena
   */
  public MeshSimulation3D(
      String meshName,
      double massKg,
      Pose3d initialPose,
      Supplier<Pose3d> originPoseSupplier,
      Supplier<Translation3d> speedsSupplier) {
    arena = SimulatedArena3D.getInstance();
    physicsEngine = arena.getPhysicsEngine();
    shape = physicsEngine.createCompoundShapeFromMesh("meshes/" + meshName);
    // TODO: add option to make btCompoundShape that moves together with another mesh or shape (ex.
    // drive)
    body = physicsEngine.createDynamicBody(shape, massKg, initialPose);
    this.originPoseSupplier = originPoseSupplier;
    arena.addCustomSimulation(this);
    this.speedsSupplier = speedsSupplier;
  }

  @Override
  public void simulationSubTick(int subTickNum) {}

  public void setPose3d(Pose3d pose) {
    body.setPose3d(pose);
  }

  public void setAngularVelocityRadPerSec(Translation3d angularVelocityRadPerSec) {
    body.setAngularVelocityRadPerSec(angularVelocityRadPerSec);
  }

  public void setAngularPositionRad(Translation3d angularPositionRad) {
    Pose3d currentPose = getPose3d();
    Rotation3d currentRotation = currentPose.getRotation();
    body.setPose3d(
        new Pose3d(
            currentPose.getTranslation(),
            new Rotation3d(
                currentRotation.getX(), currentRotation.getY(), angularPositionRad.getZ())));
  }

  public Pose3d getPose3d() {
    return body.getPose3d();
  }

  public PhysicsBody getBody() {
    return body;
  }
}
