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

package org.austincans.frc2026.subsystems.hood;

import edu.wpi.first.math.interpolation.InterpolatingDoubleTreeMap;

public class HoodConstants {
  public static final InterpolatingDoubleTreeMap angleToPWM = new InterpolatingDoubleTreeMap();
  public static final InterpolatingDoubleTreeMap pwmToAngle = new InterpolatingDoubleTreeMap();

  static {
    // angle is measured from being placed on
    // the metal right above the linear actuator
    // subtract all angles from 90 to get shot angle
    angleToPWM.put(90 - 10.3, 1000.0);
    angleToPWM.put(90 - 12.1, 1050.0);
    angleToPWM.put(90 - 13.9, 1100.0);
    angleToPWM.put(90 - 15.6, 1150.0);
    angleToPWM.put(90 - 17.3, 1200.0);
    angleToPWM.put(90 - 18.8, 1250.0);
    angleToPWM.put(90 - 20.7, 1300.0);
    angleToPWM.put(90 - 22.5, 1350.0);
    angleToPWM.put(90 - 23.7, 1400.0);
    angleToPWM.put(90 - 24.8, 1450.0);

    angleToPWM.put(90 - 28.7, 1500.0);
    angleToPWM.put(90 - 30.6, 1550.0);
    angleToPWM.put(90 - 32.4, 1600.0);
    angleToPWM.put(90 - 34.3, 1650.0);
    angleToPWM.put(90 - 36.15, 1700.0);

    pwmToAngle.put(1000.0, 90 - 10.3);
    pwmToAngle.put(1050.0, 90 - 12.1);
    pwmToAngle.put(1100.0, 90 - 13.9);
    pwmToAngle.put(1150.0, 90 - 15.6);
    pwmToAngle.put(1200.0, 90 - 17.3);
    pwmToAngle.put(1250.0, 90 - 18.8);
    pwmToAngle.put(1300.0, 90 - 20.7);
    pwmToAngle.put(1350.0, 90 - 22.5);
    pwmToAngle.put(1400.0, 90 - 23.7);
    pwmToAngle.put(1450.0, 90 - 24.8);

    pwmToAngle.put(1500.0, 90 - 28.7);
    pwmToAngle.put(1550.0, 90 - 30.6);
    pwmToAngle.put(1600.0, 90 - 32.4);
    pwmToAngle.put(1650.0, 90 - 34.3);
    pwmToAngle.put(1700.0, 90 - 36.15);
  }
}
