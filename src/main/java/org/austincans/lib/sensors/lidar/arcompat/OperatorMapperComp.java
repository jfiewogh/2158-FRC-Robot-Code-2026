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

<<<<<<<< HEAD:src/main/java/org/austincans/lib/lidar/arcompat/RegisterBinding.java
package org.austincans.lib.lidar.arcompat;
========
package org.austincans.lib.sensors.lidar.arcompat;
>>>>>>>> develop:src/main/java/org/austincans/frc2026/input/OperatorMapperComp.java

import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.Trigger;

public class OperatorMapperComp implements OperatorMapper {
  protected final CommandXboxController controller =
      new CommandXboxController(MapperConstants.operatorPort);

  @Override
  public double getRotationX() {
    return -controller.getRightY();
  }

  @Override
  public double getRotationY() {
    return -controller.getRightX();
  }

  @Override
  public Trigger getOuttakeButton() {
    return controller.leftBumper();
  }

  @Override
  public Trigger getStaticShooterToggleButton() {
    return controller.rightBumper();
  }

  @Override
  public Trigger getResetTurretAngleButton() {
    return controller.y();
  }

  @Override
  public Trigger getSetLowestHoodAngleButton() {
    return controller.x();
  }
}
