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

import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.Trigger;

public class DriverMapperComp implements DriverMapper {
  private final CommandXboxController controller =
      new CommandXboxController(MapperConstants.driverPort);

  @Override
  public double getTranslationX() {
    return -controller.getLeftY();
  }

  @Override
  public double getTranslationY() {
    return -controller.getLeftX();
  }

  @Override
  public double getRotationX() {
    return -controller.getRightY();
  }

  @Override
  public double getRotationY() {
    return -controller.getRightX();
  }

  @Override
  public Trigger getShootButton() {
    return controller.rightBumper();
  }

  @Override
  public Trigger getRollerToggleButton() {
    return controller.leftBumper();
  }

  @Override
  public Trigger getTurnModeToggleButton() {
    return controller.b();
  }

  @Override
  public Trigger getDeployButton() {
    return controller.leftTrigger();
  }

  @Override
  public Trigger getRetractButton() {
    return controller.rightTrigger();
  }

  @Override
  public Trigger getResetPoseButton() {
    return controller.back();
  }

  @Override
  public Trigger getStopWithXButton() {
    return controller.x();
  }
}
