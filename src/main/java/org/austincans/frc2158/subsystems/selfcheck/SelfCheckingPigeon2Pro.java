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

import com.ctre.phoenix6.StatusCode;
import com.ctre.phoenix6.StatusSignal;
import com.ctre.phoenix6.hardware.Pigeon2;
import java.util.ArrayList;
import java.util.List;

public class SelfCheckingPigeon2Pro implements SelfChecking {
  private final String label;
  private final StatusSignal<Integer> firmwareVersionSignal;
  private final StatusSignal<Boolean> hardwareFaultSignal;
  private final StatusSignal<Boolean> bootEnabledSignal;
  private final StatusSignal<Boolean> bootMotionSignal;
  private final StatusSignal<Boolean> accelFaultSignal;
  private final StatusSignal<Boolean> gyroFaultSignal;

  public SelfCheckingPigeon2Pro(String label, Pigeon2 pigeon) {
    this.label = label;

    this.firmwareVersionSignal = pigeon.getVersion();
    this.hardwareFaultSignal = pigeon.getFault_Hardware();
    this.bootEnabledSignal = pigeon.getFault_BootDuringEnable();
    this.bootMotionSignal = pigeon.getFault_BootIntoMotion();
    this.accelFaultSignal = pigeon.getFault_BootupAccelerometer();
    this.gyroFaultSignal = pigeon.getFault_BootupGyroscope();
  }

  @Override
  public List<SubsystemFault> checkForFaults() {
    List<SubsystemFault> faults = new ArrayList<>();

    if (firmwareVersionSignal.refresh().getStatus() != StatusCode.OK) {
      faults.add(new SubsystemFault(String.format("[%s]: No communication with device", label)));
    }
    if (hardwareFaultSignal.refresh().getValue()) {
      faults.add(new SubsystemFault(String.format("[%s]: Hardware fault detected", label)));
    }
    if (bootEnabledSignal.refresh().getValue()) {
      faults.add(new SubsystemFault(String.format("[%s]: Device booted while enabled", label)));
    }
    if (bootMotionSignal.refresh().getValue()) {
      faults.add(new SubsystemFault(String.format("[%s]: Device booted while in motion", label)));
    }
    if (accelFaultSignal.refresh().getValue()) {
      faults.add(
          new SubsystemFault(String.format("[%s]: Accelerometer boot checks failed", label)));
    }
    if (gyroFaultSignal.refresh().getValue()) {
      faults.add(new SubsystemFault(String.format("[%s]: Gyro boot checks failed", label)));
    }

    return faults;
  }
}
