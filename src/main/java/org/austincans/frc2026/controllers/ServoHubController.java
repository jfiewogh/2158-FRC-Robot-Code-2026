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

package org.austincans.frc2026.controllers;

import com.revrobotics.ResetMode;
import com.revrobotics.servohub.ServoChannel;
import com.revrobotics.servohub.ServoChannel.ChannelId;
import com.revrobotics.servohub.ServoHub;
import com.revrobotics.servohub.config.ServoChannelConfig;
import com.revrobotics.servohub.config.ServoHubConfig;

public class ServoHubController {
  public ServoHub hub = new ServoHub(44);
  ServoHubConfig config = new ServoHubConfig();

  public ServoHubController() {
    config
        .channel0
        .pulseRange(1000, 1350, 1700)
        .disableBehavior(ServoChannelConfig.BehaviorWhenDisabled.kSupplyPower);

    config
        .channel2
        .pulseRange(1000, 1350, 1700)
        .disableBehavior(ServoChannelConfig.BehaviorWhenDisabled.kSupplyPower);
    hub.configure(config, ResetMode.kNoResetSafeParameters);
  }

  private final ServoChannel getServoChannel(ChannelId id) {
    return hub.getServoChannel(id);
  }

  public void setPulse(ChannelId id, int pulseMicroSec) {
    ServoChannel servoChannel = getServoChannel(id);
    servoChannel.setEnabled(true);
    servoChannel.setPowered(true);
    servoChannel.setPulseWidth(pulseMicroSec);
  }

  public int getPulseWidth(ChannelId id) {
    return getServoChannel(id).getPulseWidth();
  }
}
