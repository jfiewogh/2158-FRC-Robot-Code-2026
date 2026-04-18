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

import com.revrobotics.servohub.ServoChannel.ChannelId;
import org.austincans.lib.controllers.ServoHubController;
import org.austincans.lib.subsystems.selfcheck.AdvancedSubsystem;

public class HoodIOServoLinear implements HoodIO {
  private final ServoHubController servoHub = new ServoHubController();
  private final ChannelId id = ChannelId.kChannelId0;

  public HoodIOServoLinear() {
    setPulse(1000);
  }

  @Override
  public void updateInputs(HoodIOInputs inputs) {
    inputs.pulseWidth = servoHub.getPulseWidth(id);
  }

  public void setPulse(int pulseWidth) {
    servoHub.setPulse(id, pulseWidth);
  }

  @Override
  public void applyOutputs(HoodIOOutputs outputs) {
    setPulse(outputs.pulseWidth);
  }

  @Override
  public void registerSelfCheckHardware(AdvancedSubsystem subsystem) {}
}
