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
// Updated on Saturday February 21, 2026 at 16:51:46 CST

package org.ironmaple.simulation.seasonspecific.rebuilt2026;

import static edu.wpi.first.units.Units.Inches;
import static edu.wpi.first.units.Units.Meters;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.Distance;
import edu.wpi.first.units.measure.LinearVelocity;
import org.ironmaple.simulation.gamepieces.GamePieceProjectile;

/**
 *
 *
 * <h1>Represents an FUEL launched into the air.</h1>
 *
 * <p>This class models a {@link RebuiltFuelOnField} launched into the air.
 *
 * <p>The simulation will determine if the FUEL hits its target—the HUB.
 *
 * <p>The user can specify a callback using {@link #setHitNetCallBack(Runnable)}, which will be
 * triggered when the FUEL hits the HUB.
 */
public class RebuiltFuelOnFly extends GamePieceProjectile {

  public RebuiltFuelOnFly(
      Translation2d robotPosition,
      Translation2d shooterPositionOnRobot,
      ChassisSpeeds chassisSpeeds,
      Rotation2d shooterFacing,
      Distance initialHeight,
      LinearVelocity launchingSpeed,
      Angle shooterAngle) {
    super(
        RebuiltFuelOnField.REBUILT_FUEL_INFO,
        robotPosition,
        shooterPositionOnRobot,
        chassisSpeeds,
        shooterFacing,
        initialHeight,
        launchingSpeed,
        shooterAngle);

    super.withTouchGroundHeight(Inches.of(3).in(Meters));
    super.enableBecomesGamePieceOnFieldAfterTouchGround();
  }
}
