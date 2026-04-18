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

<<<<<<<< HEAD:src/main/java/org/austincans/lib/util/Deadband.java
package org.austincans.lib.util;
========
package org.austincans.frc2026.input;
>>>>>>>> develop:src/main/java/org/austincans/frc2026/input/DriverMapper.java

import edu.wpi.first.wpilibj2.command.button.Trigger;

public interface DriverMapper {
  public double getTranslationX();

  public double getTranslationY();

  public double getRotationX();

  public double getRotationY();

  public Trigger getShootButton();

  public Trigger getRollerToggleButton();

  public Trigger getTurnModeToggleButton();

  /* Just for real/comp */
  public default Trigger getDeployButton() {
    return new Trigger(() -> false);
  }

  public default Trigger getRetractButton() {
    return new Trigger(() -> false);
  }

  public default Trigger getZeroPoseButton() {
    return new Trigger(() -> false);
  }

  public default Trigger getStopWithXButton() {
    return new Trigger(() -> false);
  }
}
