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

package org.austincans.frc2026.subsystems;

import java.util.Optional;
import org.austincans.frc2026.Constants;
import org.austincans.frc2026.RobotContainer;
import org.austincans.frc2026.generated.TunerConstants;
import org.austincans.frc2026.subsystems.drive.Drive;
import org.austincans.frc2026.subsystems.drive.GyroIOFake;
import org.austincans.frc2026.subsystems.drive.GyroIOPigeon2;
import org.austincans.frc2026.subsystems.drive.ModuleIOFake;
import org.austincans.frc2026.subsystems.drive.ModuleIOTalonFXReal;
import org.austincans.frc2026.subsystems.feeder.Indexer;
import org.austincans.frc2026.subsystems.feeder.IndexerIOSparkFlex;
import org.austincans.frc2026.subsystems.feeder.Spindexer;
import org.austincans.frc2026.subsystems.feeder.SpindexerIOSparkFlex;
import org.austincans.frc2026.subsystems.flywheel.Flywheel;
import org.austincans.frc2026.subsystems.flywheel.FlywheelIOTalonFX;
import org.austincans.frc2026.subsystems.hood.Hood;
import org.austincans.frc2026.subsystems.hood.HoodIOServoLinear;
import org.austincans.frc2026.subsystems.intake.deploy.Deploy;
import org.austincans.frc2026.subsystems.intake.deploy.DeployIOSparkFlex;
import org.austincans.frc2026.subsystems.intake.roller.Roller;
import org.austincans.frc2026.subsystems.intake.roller.RollerIOSparkFlex;
import org.austincans.frc2026.subsystems.led.Led;
import org.austincans.frc2026.subsystems.led.LedIO;
import org.austincans.frc2026.subsystems.led.LedIOBlinkin;
import org.austincans.frc2026.subsystems.turret.Turret;
import org.austincans.frc2026.subsystems.turret.TurretIOSparkFlex;
import org.austincans.frc2026.subsystems.vision.ObjectDetection;
import org.austincans.frc2026.subsystems.vision.ObjectDetectionIOReal;
import org.austincans.frc2026.subsystems.vision.Vision;
import org.austincans.frc2026.subsystems.vision.VisionConstants;
import org.austincans.frc2026.subsystems.vision.VisionConstants.LimelightConfig;
import org.austincans.frc2026.subsystems.vision.VisionIO;
import org.austincans.frc2026.subsystems.vision.VisionIOLimelight;

public class SubsystemFactory {
  public static Optional<Drive> buildDrive() {
    if (!RobotContainer.driveEnabled) {
      return Optional.empty();
    }
    Drive drive;
    if (Constants.currentMode == Constants.Mode.REAL) {
      drive =
          new Drive(
              new GyroIOPigeon2(),
              new ModuleIOTalonFXReal(TunerConstants.FrontLeft),
              new ModuleIOTalonFXReal(TunerConstants.FrontRight),
              new ModuleIOTalonFXReal(TunerConstants.BackLeft),
              new ModuleIOTalonFXReal(TunerConstants.BackRight),
              (pose) -> {});
    } else {
      drive =
          new Drive(
              new GyroIOFake(),
              new ModuleIOFake(),
              new ModuleIOFake(),
              new ModuleIOFake(),
              new ModuleIOFake(),
              (pose) -> {});
    }
    return Optional.ofNullable(drive);
  }

  public static Optional<Vision> buildVision(Optional<Drive> drive) {
    if (drive.isEmpty()) {
      return Optional.empty();
    }
    if (!RobotContainer.visionEnabled) {
      return Optional.empty();
    }
    VisionIO[] cameras;
    if (Constants.currentMode == Constants.Mode.REAL) {
      cameras =
          new VisionIO[] {
            new VisionIOLimelight(VisionConstants.camera0Name, drive.get()::getRotation),
            new VisionIOLimelight(VisionConstants.camera1Name, drive.get()::getRotation)
          };
    } else {
      cameras = new VisionIO[] {new VisionIO() {}, new VisionIO() {}};
    }
    return Optional.ofNullable(new Vision(drive.get(), cameras));
  }

  public static Optional<Turret> buildTurret() {
    if (!RobotContainer.turretEnabled) {
      return Optional.empty();
    }
    Turret turret = null;
    if (Constants.currentMode == Constants.Mode.REAL) {
      turret = new Turret(new TurretIOSparkFlex());
    }
    return Optional.ofNullable(turret);
  }

  public static Optional<Hood> buildHood() {
    if (!RobotContainer.hoodEnabled) {
      return Optional.empty();
    }
    Hood hood = null;
    if (Constants.currentMode == Constants.Mode.REAL) {
      hood = new Hood(new HoodIOServoLinear());
    }
    return Optional.ofNullable(hood);
  }

  public static Optional<Roller> buildRoller() {
    if (!RobotContainer.rollerEnabled) {
      return Optional.empty();
    }
    Roller roller = null;
    if (Constants.currentMode == Constants.Mode.REAL) {
      roller = new Roller(new RollerIOSparkFlex());
    }
    return Optional.ofNullable(roller);
  }

  public static Optional<Deploy> buildDeploy() {
    if (!RobotContainer.deployEnabled) {
      return Optional.empty();
    }
    Deploy deploy = null;
    if (Constants.currentMode == Constants.Mode.REAL) {
      deploy = new Deploy(new DeployIOSparkFlex());
    }
    return Optional.ofNullable(deploy);
  }

  public static Optional<Flywheel> buildFlywheel() {
    if (!RobotContainer.flywheelEnabled) {
      return Optional.empty();
    }
    Flywheel flywheel = null;
    if (Constants.currentMode == Constants.Mode.REAL) {
      flywheel = new Flywheel(new FlywheelIOTalonFX());
    }
    return Optional.ofNullable(flywheel);
  }

  public static Optional<Indexer> buildIndexer() {
    if (!RobotContainer.indexerEnabled) {
      return Optional.empty();
    }
    Indexer indexer = null;
    if (Constants.currentMode == Constants.Mode.REAL) {
      indexer = new Indexer(new IndexerIOSparkFlex());
    }
    return Optional.ofNullable(indexer);
  }

  public static Optional<Spindexer> buildSpindexer() {
    if (!RobotContainer.spindexerEnabled) {
      return Optional.empty();
    }
    Spindexer spindexer = null;
    if (Constants.currentMode == Constants.Mode.REAL) {
      spindexer = new Spindexer(new SpindexerIOSparkFlex());
    }
    return Optional.ofNullable(spindexer);
  }

  public static Optional<ObjectDetection> buildObjectDetection() {
    if (!RobotContainer.objectDetectionEnabled) {
      return Optional.empty();
    }
    ObjectDetection objectDetection = null;
    if (Constants.currentMode == Constants.Mode.REAL) {
      objectDetection =
          new ObjectDetection(
              new ObjectDetectionIOReal(
                  "limelight", ObjectDetection.GamePieceType.FUEL, LimelightConfig.LIMELIGHT_3G));
    }
    return Optional.ofNullable(objectDetection);
  }

  public static Optional<Led> buildLed() {
    if (!RobotContainer.ledEnabled) {
      return Optional.empty();
    }
    LedIO io = null;
    if (Constants.currentMode == Constants.Mode.REAL) {
      io = new LedIOBlinkin();
    }
    return Optional.ofNullable(new Led(io));
  }
}
