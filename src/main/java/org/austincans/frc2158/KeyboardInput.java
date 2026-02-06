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
// Updated on Thursday February 05, 2026 at 19:51:21 CST

package org.austincans.frc2158;

import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;

public class KeyboardInput {
  private final Joystick joystick0;
  private final Joystick joystick1;
  private final Joystick joystick2;

  public KeyboardInput() {
    joystick0 = new Joystick(0);
    joystick1 = new Joystick(1);
    joystick2 = new Joystick(2);
  }

  public JoystickButton z() {
    return new JoystickButton(joystick0, 1);
  }

  public JoystickButton x() {
    return new JoystickButton(joystick0, 2);
  }

  public JoystickButton c() {
    return new JoystickButton(joystick0, 3);
  }

  public JoystickButton v() {
    return new JoystickButton(joystick0, 4);
  }

  public double leftX() {
    return -joystick0.getRawAxis(0);
  }

  public double leftY() {
    return -joystick0.getRawAxis(1);
  }

  public double rightX() {
    return -joystick2.getRawAxis(0);
  }

  public double rightY() {
    return -joystick2.getRawAxis(1);
  }
}
