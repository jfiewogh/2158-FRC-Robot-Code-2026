// Copyright (c) 2025 the ausTIN CANs
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
// Updated on Wednesday November 12, 2025 at 18:18:23 CST

package org.austincans.frc2158.subsystems.selfcheck;

import edu.wpi.first.wpilibj.Timer;

public class SubsystemFault {
  public final String description;
  public final double timestamp;
  public final boolean isWarning;
  public final boolean sticky;

  public SubsystemFault(String description, boolean isWarning) {
    this(description, isWarning, false);
  }

  public SubsystemFault(String description) {
    this(description, false);
  }

  public SubsystemFault(String description, boolean isWarning, boolean sticky) {
    this.description = description;
    this.timestamp = Timer.getFPGATimestamp();
    this.isWarning = isWarning;
    this.sticky = sticky;
  }

  @Override
  public boolean equals(Object other) {
    if (other == this) {
      return true;
    }

    if (other instanceof SubsystemFault) {
      SubsystemFault o = (SubsystemFault) other;

      return description.equals(o.description) && isWarning == o.isWarning;
    }
    return false;
  }
}
