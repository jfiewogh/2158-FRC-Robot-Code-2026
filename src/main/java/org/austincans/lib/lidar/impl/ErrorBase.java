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

<<<<<<<< HEAD:src/main/java/org/austincans/lib/lidar/impl/ErrorBase.java
package org.austincans.lib.lidar.impl;

import org.austincans.lib.lidar.api.Error;
========
package org.austincans.frc2026.input;

import edu.wpi.first.wpilibj2.command.button.Trigger;
>>>>>>>> develop:src/main/java/org/austincans/frc2026/input/OperatorMapperTest.java

public class OperatorMapperTest extends OperatorMapperComp {
  @Override
  public Trigger getPointToHubButton() {
    return controller.povDown();
  }

  @Override
  public Trigger getIncrementHoodAngleButton() {
    return controller.b();
  }

  @Override
  public Trigger getDecrementHoodAngleButton() {
    return controller.a();
  }

  @Override
  public Trigger getRunIndexerAndFlywheelButton() {
    return controller.povUp();
  }
}
