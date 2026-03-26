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

import edu.wpi.first.math.geometry.Rotation2d;
import java.util.List;
import org.austincans.frc2026.subsystems.selfcheck.HardwareFault;
import org.austincans.frc2026.subsystems.selfcheck.SelfChecking;
import org.littletonrobotics.junction.AutoLog;

/* This is specifically for the linear actuator */
public abstract interface HoodIO {
  @AutoLog
  public class HoodIOInputs {
    public int pulseWidth;
    public Rotation2d hoodAngle; // angle created at pivot point
    public Rotation2d
        shotPitchAngle; // angle that the game piece is shot (relative to turret) (0 is straight
    // forward, 90 is straight up)
    public double temperatureCelsius;
    public double motorPositionRad;
    public double velocityRadPerSec;
  }

  public void updateInputs(HoodIOInputs inputs);

  public void setHoodAngle(Rotation2d angle);

  public void setPulse(int pulseWidth);

  public List<HardwareFault> checkForFaults();

  public List<SelfChecking> getHardware();
}
