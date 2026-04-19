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

package org.austincans.lib.subsystems.turret;

import com.ctre.phoenix6.CANBus;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.math.util.Units;
import java.util.function.DoubleSupplier;
import org.austincans.lib.util.LoggedTunableNumber;

public class TurretConstants {
  // values used by SparkFlex and TalonFX
  public static final DoubleSupplier kP = new LoggedTunableNumber("Turret/kP", 0.0);
  public static final DoubleSupplier kI = new LoggedTunableNumber("Turret/kI", 0.0);
  public static final DoubleSupplier kD = new LoggedTunableNumber("Turret/kD", 0.0);
  public static final DoubleSupplier kV = new LoggedTunableNumber("Turret/kV", 0.0);
  public static final DoubleSupplier kS = new LoggedTunableNumber("Turret/kS", 0.0);
  public static final DoubleSupplier kA = new LoggedTunableNumber("Turret/kA", 0.0);

  public static final int motorId = 24;
  public static final double gearing = 10; // 10:1 motor to turret rotations

  public static final double maxVelocityRotationsPerSec = 0.2;
  public static final double maxAccelerationRotationsPerSecSq = 1;

  public static final boolean forwardSoftLimitEnabled = true;
  public static final boolean reverseSoftLimitEnabled = true;
  public static final double forwardSoftLimit = 3.6 - 0.453;
  public static final double reverseSoftLimit = -3.6 - 0.453;
  public static final boolean isBrakeMode = false;

  public static final int encoderId = 25;
  public static final double magnetOffset = 0.01513671875;

  public static final double atSetpointThresholdRad = 0.1;

  // values only used by TalonFX
  public static final CANBus canBus = new CANBus("2158_CAN");
  public static final CANBus cancoderCANBus = new CANBus("rio");

  // values only used by SparkFlex
  public static final boolean isBrushlessMotor = true;
  public static final int smartCurrentLimit = 80;

  // values (possibly only) used in sim
  public static final double rotationalInertiaMetersSq = 0.01;

  public static final double turretMinimumRotation = -2.8 - 0.453;
  public static final double turretMaxRotation = 3.6 - 0.453;

  // REAL
  public static final Transform3d robotRelTurretTransform = // relative to robot zero
      new Transform3d(
          -0.123, // negative because turret is toward the back
          0,
          Units.inchesToMeters(18.375 + 5.5), // camera height
          new Rotation3d(Rotation2d.k180deg));

  /** The difference between the robot's zero position and the turret's zero position */
  public static final Rotation2d robotRelTurretRotationOffset =
      robotRelTurretTransform.getRotation().toRotation2d();

  public static final Translation3d fieldRelTurretTranslationOffset =
      robotRelTurretTransform.getTranslation().rotateBy(robotRelTurretTransform.getRotation());

  public static final double heightMeters = robotRelTurretTransform.getZ();
}