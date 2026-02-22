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
// Updated on Saturday February 21, 2026 at 23:31:39 CST

package org.austincans.frc2158.controllers;

import edu.wpi.first.wpilibj.Servo;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

// @author Austin Cans Robotics Team 2158
public class LinearActuatorController extends SubsystemBase {
  private static Servo linearActuator;
  private REVDigitController controller = new REVDigitController();
  private double speed = 0; // PWM in ms

  // L16-100-35-6-R
  public LinearActuatorController(int linearActuatorDeviceID) {
    linearActuator = new Servo(linearActuatorDeviceID);
    linearActuator.setBoundsMicroseconds(2, 0, 1, 0, 1);
  }

  public void setSpeed(double speed) {
    this.speed = speed;
  }

  public double getPWM() {
    return speed;
  }

  @Override
  public void periodic() {
    if (speed == 0) {
      setSpeed(controller.getInstance().getPotentiometer() + 1);
    } else {
      setSpeed(speed);
    }
  }
}
