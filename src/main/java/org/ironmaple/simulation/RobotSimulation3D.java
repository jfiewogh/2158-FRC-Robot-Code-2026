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

import com.jme3.bullet.collision.PhysicsCollisionObject;
import com.jme3.bullet.joints.SixDofJoint;
import com.jme3.math.Vector3f;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.geometry.Translation3d;
import org.austincans.frc2158.subsystems.drive.DriveConstants;
import org.austincans.frc2158.subsystems.sim.RebuiltShooterViz;
import org.austincans.frc2158.subsystems.turret.TurretConstants;
import org.ironmaple.simulation.drivesims.AbstractDriveTrainSimulation3D;
import org.ironmaple.simulation.physics.PhysicsShape;
import org.ironmaple.simulation.physics.bullet.BulletBody;
import org.ironmaple.simulation.physics.bullet.BulletPhysicsEngine;
import org.littletonrobotics.junction.Logger;

// Works only for Bullet, not Jolt
public class RobotSimulation3D implements SimulatedArena3D.Simulatable {
  private final String robotName;

  private final SimulatedArena3D arena;
  private final BulletPhysicsEngine physicsEngine;
  private final AbstractDriveTrainSimulation3D driveSimulation;

  private IntakeSimulation3D intakeSimulation;
  // shooter
  private ShooterSimulation3D shooterSimulation;
  private MeshSimulation3D shooterBase;
  private MeshSimulation3D turret;
  private MeshSimulation3D hood;

  public RobotSimulation3D(
      String robotName, AbstractDriveTrainSimulation3D driveSimulation, Pose2d initialPose) {
    this.robotName = robotName;
    this.driveSimulation = driveSimulation;
    driveSimulation.registerWithArena(initialPose);

    arena = SimulatedArena3D.getInstance();
    physicsEngine = (BulletPhysicsEngine) arena.getPhysicsEngine();
    arena.addCustomSimulation(this);
  }

  // TODO: IntakeSimulation3D has no collision, add it
  // GamePiece and type should come from an enum (ex. from ObjectDetection)
  // If number of game pieces in intake is greater than max (ex. 35), it should
  // exit out the robot
  public void addIntakeSimulation(
      String targetedGamePieceType,
      double intakeDepth,
      double intakeWidth,
      double intakeHeight,
      int capacity) {
    PhysicsShape intakeShape =
        physicsEngine.createBoxShape(
            new Translation3d(intakeDepth * 0.5, intakeWidth * 0.5, intakeHeight * 0.5));
    Transform3d intakeOffset =
        new Transform3d(
            DriveConstants.trackWidthX / 2 + intakeDepth / 2,
            0,
            intakeHeight / 2,
            Rotation3d.kZero);
    intakeSimulation =
        new IntakeSimulation3D("Fuel", driveSimulation, intakeShape, intakeOffset, capacity);
  }

  public void addShooterSimulation(ShooterSimulation3D shooterSimulation) {}

  public void addShooterBase(MeshSimulation3D shooterBase) {
    this.shooterBase = shooterBase;
    shooterBase.setPose3d(
        driveSimulation
            .getSimulatedDriveTrainPose3dGroundRelative()
            .plus(new Transform3d(0, 0, TurretConstants.turretHeightMeters, Rotation3d.kZero)));

    BulletBody driveSimulationBody = (BulletBody) driveSimulation.getPhysicsBody();
    BulletBody shooterBaseBody = (BulletBody) shooterBase.getBody();

    SixDofJoint sixDofJoint =
        new SixDofJoint(
            driveSimulationBody.getRigidBody(),
            shooterBaseBody.getRigidBody(),
            new Vector3f(0, 0, (float) TurretConstants.turretHeightMeters),
            Vector3f.ZERO,
            true);
    sixDofJoint.setAngularUpperLimit(Vector3f.ZERO);
    sixDofJoint.setAngularLowerLimit(Vector3f.ZERO);
    sixDofJoint.setLinearLowerLimit(Vector3f.ZERO);
    sixDofJoint.setLinearUpperLimit(Vector3f.ZERO);
    sixDofJoint.setCollisionBetweenLinkedBodies(false);
    physicsEngine.getPhysicsSpace().addJoint(sixDofJoint);
  }

  public void addTurret(MeshSimulation3D turret) {
    this.turret = turret;
    turret.setPose3d(shooterBase.getPose3d());

    BulletBody shooterBaseBody = (BulletBody) shooterBase.getBody();
    BulletBody turretBody = (BulletBody) turret.getBody();

    SixDofJoint sixDofJoint =
        new SixDofJoint(
            shooterBaseBody.getRigidBody(),
            turretBody.getRigidBody(),
            Vector3f.ZERO,
            Vector3f.ZERO,
            true);
    sixDofJoint.setAngularUpperLimit(new Vector3f(0, 0, -1));
    sixDofJoint.setAngularLowerLimit(new Vector3f(0, 0, 1));
    sixDofJoint.setLinearLowerLimit(Vector3f.ZERO);
    sixDofJoint.setLinearUpperLimit(Vector3f.ZERO);
    sixDofJoint.setCollisionBetweenLinkedBodies(false);
    physicsEngine.getPhysicsSpace().addJoint(sixDofJoint);
  }

  // TODO: Fix hood mesh simulation. Pitch does not change
  public void addHood(MeshSimulation3D hood) {
    this.hood = hood;
    hood.setPose3d(
        shooterBase
            .getPose3d()
            .plus(new Transform3d(RebuiltShooterViz.modelTranslations[2], Rotation3d.kZero)));

    BulletBody shooterBaseBody = (BulletBody) shooterBase.getBody();
    BulletBody turretBody = (BulletBody) turret.getBody();
    BulletBody hoodBody = (BulletBody) hood.getBody();
    hoodBody
        .getRigidBody()
        .setIgnoreList(
            new PhysicsCollisionObject[] {
              shooterBaseBody.getRigidBody(), turretBody.getRigidBody()
            });

    SixDofJoint sixDofJoint =
        new SixDofJoint(
            turretBody.getRigidBody(),
            hoodBody.getRigidBody(),
            Vector3f.ZERO,
            // new Vector3f(
            //     (float) RebuiltShooterViz.modelTranslations[2].getX(),
            //     (float) RebuiltShooterViz.modelTranslations[2].getY(),
            //     (float) RebuiltShooterViz.modelTranslations[2].getZ()),
            Vector3f.ZERO,
            true);
    sixDofJoint.setAngularUpperLimit(new Vector3f(0, (float) -Math.PI / 2, (float) Math.PI / 4));
    sixDofJoint.setAngularLowerLimit(new Vector3f(0, 0, 0));
    sixDofJoint.setLinearLowerLimit(Vector3f.ZERO);
    sixDofJoint.setLinearUpperLimit(Vector3f.ZERO);
    sixDofJoint.setCollisionBetweenLinkedBodies(false);
    physicsEngine.getPhysicsSpace().addJoint(sixDofJoint);
  }

  @Override
  public void simulationSubTick(int subTickNum) {
    Logger.recordOutput("RobotSimulation/ShooterBasePose", shooterBase.getPose3d());
    Logger.recordOutput("RobotSimulation/TurretPose", turret.getPose3d());
    Logger.recordOutput("RobotSimulation/HoodPose", hood.getPose3d());
  }

  // Getters

  public AbstractDriveTrainSimulation3D getDriveSimulation() {
    return driveSimulation;
  }

  public IntakeSimulation3D getIntakeSimulation() {
    return intakeSimulation;
  }

  public MeshSimulation3D getShooterBase() {
    return shooterBase;
  }

  public MeshSimulation3D getTurret() {
    return turret;
  }

  public MeshSimulation3D getHood() {
    return hood;
  }

  public Pose3d getDriveTrainPose() {
    return driveSimulation.getSimulatedDriveTrainPose3dGroundRelative();
  }
}
