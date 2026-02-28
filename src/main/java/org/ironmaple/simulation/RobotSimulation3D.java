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

import com.jme3.bullet.joints.HingeJoint;
import com.jme3.math.Vector3f;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform3d;
import java.util.ArrayList;
import org.austincans.frc2158.subsystems.turret.TurretConstants;
import org.ironmaple.simulation.drivesims.AbstractDriveTrainSimulation3D;
import org.ironmaple.simulation.physics.bullet.BulletBody;
import org.ironmaple.simulation.physics.bullet.BulletPhysicsEngine;

// Works only for Bullet, not Jolt
public class RobotSimulation3D implements SimulatedArena3D.Simulatable {
  private final SimulatedArena3D arena;
  private final BulletPhysicsEngine physicsEngine;
  private final AbstractDriveTrainSimulation3D driveTrainSimulation;
  private ArrayList<MeshSimulation3D> meshSimulations;
  private IntakeSimulation3D intakeSimulation;
  private ShooterSimulation3D shooterSimulation;
  private MeshSimulation3D turretSimulation;

  // TODO: maybe put all simulatables in one ArrayList, including
  // driveTrainSimulation because it is
  // also a simulatable
  // This can help with simplification but be a challenge if we want to access
  // and/or modify a
  // specific simulatable

  public RobotSimulation3D(AbstractDriveTrainSimulation3D driveTrainSimulation) {
    this.driveTrainSimulation = driveTrainSimulation;
    arena = SimulatedArena3D.getInstance();
    physicsEngine = (BulletPhysicsEngine) arena.getPhysicsEngine();
  }

  public void addMeshSimulation(MeshSimulation3D meshSimulation) {
    meshSimulations.add(meshSimulation);
  }

  public void addTurretSimulation(MeshSimulation3D turretSimulation) {
    this.turretSimulation = turretSimulation;
    turretSimulation.setPose3d(
        driveTrainSimulation
            .getSimulatedDriveTrainPose3d()
            .plus(new Transform3d(0, 0, TurretConstants.turretHeightMeters, Rotation3d.kZero)));
    HingeJoint joint =
        new HingeJoint(
            ((BulletBody) driveTrainSimulation.getPhysicsBody()).getRigidBody(),
            ((BulletBody) turretSimulation.getBody()).getRigidBody(),
            new Vector3f(0, 0, (float) TurretConstants.turretHeightMeters),
            new Vector3f(0, 0, 0),
            Vector3f.UNIT_Z,
            Vector3f.UNIT_Z);
    joint.setCollisionBetweenLinkedBodies(false);
    physicsEngine.getPhysicsSpace().addJoint(joint);
  }

  @Override
  public void simulationSubTick(int subTickNum) {}

  public Pose3d getDriveTrainPose() {
    return driveTrainSimulation.getSimulatedDriveTrainPose3dGroundRelative();
  }
}
