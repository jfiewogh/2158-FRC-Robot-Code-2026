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

package org.austincans.frc2026.subsystems.flywheel;

import static edu.wpi.first.units.Units.Amps;
import static edu.wpi.first.units.Units.Seconds;

import com.ctre.phoenix6.CANBus;
import edu.wpi.first.units.measure.Current;
import edu.wpi.first.units.measure.Time;
import org.austincans.lib.util.LoggedTunableNumber;

public class FlywheelConstants {
  LoggedTunableNumber FlywheelkP = new LoggedTunableNumber("Tuning/FlywheelKP", 2.6);
  LoggedTunableNumber FlywheelkI = new LoggedTunableNumber("Tuning/FlywheelKI", 0);
  LoggedTunableNumber FlywheelkD = new LoggedTunableNumber("Tuning/FlywheelKD", 0);
  LoggedTunableNumber FlywheelkS = new LoggedTunableNumber("Tuning/FlywheelKS", 0);
  LoggedTunableNumber FlywheelKV = new LoggedTunableNumber("Tuning/FlywheelKV", 0);
  public static final double flywheelGearRation = 2;
  public static final int maxDistFromLasarCan1 = 30;
  public static final int maxDistFromLasarCan2 = 30;

  /* REAL */
  public static final int motor1Id = 22;
  public static final int motor2Id = 23;
  public static final int encoderId = 0;
  public static final int lasarCan1Id = 0;
  public static final int lasarCan2Id = 1;
  public static final CANBus canBus = new CANBus("rio");

  public static final Current statorCurrentLimit = Amps.of(70);
  public static final Current supplyCurrentLimit = Amps.of(50);

  public static final Time flywheelClosedLoopRamp = Seconds.of(0.5);
  public static final Time flywheelOpenLoopRamp = Seconds.of(0.5);

  public static final double radiusMeters = 0.0508;
  public static final double gearing = 0;

  public static final double atSetpointThresholdRPM = 50.0;

  /* SIM */
  public static final double JKgMetersSq = 0;

  public static final double maxRPM = 2100;
}
