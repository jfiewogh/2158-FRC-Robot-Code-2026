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

import java.util.function.DoubleSupplier;
import org.austincans.frc2026.util.LoggedTunableNumber;

public class HoodConstants {
  static int flexMotorID = 0;
  static int talonMotorID = 0;
  static int linearActuatorID = 20;
  static DoubleSupplier kP = new LoggedTunableNumber("Hood/kP", 0);
  static DoubleSupplier kI = new LoggedTunableNumber("Hood/kI", 0);
  static DoubleSupplier kD = new LoggedTunableNumber("Hood/kD", 0);
  static DoubleSupplier kV = new LoggedTunableNumber("Hood/kV", 0);
  public static final double rotationalInertiaMetersSq = 0.01;
  public static final double gearing = 5;

  // public static final double maxAngle = ConfigUtil.getConfigDouble("Max Hood Angle", 45);
}
