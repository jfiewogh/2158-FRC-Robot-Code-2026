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

package org.austincans.frc2026.subsystems.selfcheck;

import com.ctre.phoenix6.StatusCode;
import com.ctre.phoenix6.StatusSignal;
import com.ctre.phoenix6.hardware.TalonFXS;
import java.util.ArrayList;
import java.util.List;

public class SelfCheckingTalonFXS implements SelfChecking {
  private final String label;
  private final int ID;
  private final StatusSignal<Integer> firmwareVersionSignal;
  private final StatusSignal<Boolean> hardwareFaultSignal;
  private final StatusSignal<Boolean> bootEnabledSignal;
  private final StatusSignal<Boolean> deviceTempSignal;
  private final StatusSignal<Boolean> procTempSignal;

  public SelfCheckingTalonFXS(String label, TalonFXS talon) {
    this.label = label;
    this.ID = talon.getDeviceID();

    firmwareVersionSignal = talon.getVersion();
    hardwareFaultSignal = talon.getFault_Hardware();
    bootEnabledSignal = talon.getFault_BootDuringEnable();
    deviceTempSignal = talon.getFault_DeviceTemp();
    procTempSignal = talon.getFault_ProcTemp();
  }

  @Override
  public List<HardwareFault> checkForFaults() {
    List<HardwareFault> faults = new ArrayList<>();

    if (firmwareVersionSignal.refresh().getStatus() != StatusCode.OK) {
      faults.add(
          new HardwareFault(
              "TAL" + String.valueOf(ID),
              String.format("[%s]: No communication with device", label)));
    }
    if (hardwareFaultSignal.refresh().getValue()) {
      faults.add(
          new HardwareFault(
              "TAL" + String.valueOf(ID), String.format("[%s]: Hardware fault detected", label)));
    }
    if (bootEnabledSignal.refresh().getValue()) {
      faults.add(
          new HardwareFault(
              "TAL" + String.valueOf(ID),
              String.format("[%s]: Device booted while enabled", label)));
    }
    if (deviceTempSignal.refresh().getValue()) {
      faults.add(
          new HardwareFault(
              "TAL" + String.valueOf(ID),
              String.format("[%s]: Device temperature too high", label),
              true));
    }
    if (procTempSignal.refresh().getValue()) {
      faults.add(
          new HardwareFault(
              "TAL" + String.valueOf(ID),
              String.format("[%s]: Processor temperature too high", label),
              true));
    }

    return faults;
  }
}
