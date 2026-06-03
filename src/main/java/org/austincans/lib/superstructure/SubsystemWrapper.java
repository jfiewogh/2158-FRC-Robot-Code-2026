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

package org.austincans.lib.superstructure;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public abstract class SubsystemWrapper<Subsystem extends SubsystemBase> {
  protected final boolean isPresent;
  protected final Subsystem subsystem;

  public SubsystemWrapper(boolean isPresent, Subsystem subsystem) {
    this.isPresent = isPresent;
    this.subsystem = subsystem;
  }

  public void setDefaultCommand(Command command) {
    if (isPresent) {
      subsystem.setDefaultCommand(command);
    }
  }

  /** Returns whether subsystem is non-null  */
  public boolean isPresent() {
    return isPresent;
  }
  /** Returns whether subsystem is null  */
  public boolean isEmpty() {
    return !isPresent;
  }

  /** Return instance of subsystem, could be null */
  public Subsystem getSubsystem() {
    return subsystem;
  }
}
