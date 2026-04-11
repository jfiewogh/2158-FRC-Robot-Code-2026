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

package org.austincans.frc2026.input;

import edu.wpi.first.wpilibj2.command.button.Trigger;

public interface OperatorMapper {
  public double getRotationX();

  public double getRotationY();

  public Trigger getOuttakeButton();

  public Trigger getStaticShooterToggleButton();

  public Trigger getResetTurretAngleButton();

  public Trigger getSetLowestHoodAngleButton();

  public Trigger getTrackingButton();

  /* Just for testing */
  public default Trigger getPointToHubButton() {
    return new Trigger(() -> false);
  }

  public default Trigger getIncrementHoodAngleButton() {
    return new Trigger(() -> false);
  }

  public default Trigger getDecrementHoodAngleButton() {
    return new Trigger(() -> false);
  }

  public default Trigger getRunIndexerAndFlywheelButton() {
    return new Trigger(() -> false);
  }
}
