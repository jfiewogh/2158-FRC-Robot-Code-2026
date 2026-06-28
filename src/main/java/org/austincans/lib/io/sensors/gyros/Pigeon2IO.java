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

package org.austincans.lib.io.sensors.gyros;

import com.ctre.phoenix6.BaseStatusSignal;
import com.ctre.phoenix6.CANBus;
import com.ctre.phoenix6.StatusCode;
import com.ctre.phoenix6.StatusSignal;
import com.ctre.phoenix6.configs.Pigeon2Configuration;
import com.ctre.phoenix6.hardware.Pigeon2;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.units.measure.Frequency;

import org.austincans.lib.util.PhoenixUtil;

/** IO implementation for Pigeon 2. */
public class Pigeon2IO implements GyroIO<Pigeon2IO.Pigeon2IOInputs> {
  private final Pigeon2 pigeon;

  private final StatusSignal<Angle> yaw;
  private final StatusSignal<Angle> pitch;
  private final StatusSignal<Angle> roll;
  private final StatusSignal<AngularVelocity> yawVelocity;
  private final StatusSignal<AngularVelocity> pitchVelocity;
  private final StatusSignal<AngularVelocity> rollVelocity;

  // Based on https://github.com/3015RangerRobotics/2024Public/blob/main/RobotCode2024/src/main/java/frc/lib/subsystem/selfcheck/SelfCheckingPigeon2.java
  private final StatusSignal<Integer> firmwareVersion;
  private final StatusSignal<Boolean> hardwareFault;
  private final StatusSignal<Boolean> bootDuringEnable;
  private final StatusSignal<Boolean> bootIntoMotion;
  private final StatusSignal<Boolean> bootupAccelerometer;
  private final StatusSignal<Boolean> bootupGyroscope;

  private final StatusSignal<Boolean> stickyFaultHardware;
  private final StatusSignal<Boolean> stickyFaultBootDuringEnable;
  private final StatusSignal<Boolean> stickyFaultBootIntoMotion;
  private final StatusSignal<Boolean> stickyFaultBootupAccelerometer;
  private final StatusSignal<Boolean> stickyFaultBootupGyroscope;

  private final BaseStatusSignal[] signals;

  protected Pigeon2IO(int deviceId, CANBus canBus, Frequency updateFrequency) {
    pigeon = new Pigeon2(deviceId, canBus);

    Pigeon2Configuration config = new Pigeon2Configuration();
    pigeon.getConfigurator().apply(config);
    pigeon.getConfigurator().setYaw(0.0);

    yaw = pigeon.getYaw();
    pitch = pigeon.getPitch();
    roll = pigeon.getRoll();
    yawVelocity = pigeon.getAngularVelocityZWorld();
    pitchVelocity = pigeon.getAngularVelocityXWorld();
    rollVelocity = pigeon.getAngularVelocityYWorld();

    yaw.setUpdateFrequency(updateFrequency);
    BaseStatusSignal.setUpdateFrequencyForAll(
        50, pitch, roll, yawVelocity, pitchVelocity, rollVelocity);
    pigeon.optimizeBusUtilization();
    
    PhoenixUtil.registerSignals(true, yaw, yawVelocity, pitch, pitchVelocity, roll, rollVelocity);
    PhoenixUtil.tryUntilOk(5, () -> pigeon.setYaw(0.0, 0.25));

    firmwareVersion = pigeon.getVersion();
    hardwareFault = pigeon.getFault_Hardware();
    bootDuringEnable = pigeon.getFault_BootDuringEnable();
    bootIntoMotion = pigeon.getFault_BootIntoMotion();
    bootupAccelerometer = pigeon.getFault_BootupAccelerometer();
    bootupGyroscope = pigeon.getFault_BootupGyroscope();

    stickyFaultHardware = pigeon.getStickyFault_Hardware();
    stickyFaultBootDuringEnable = pigeon.getStickyFault_BootDuringEnable();
    stickyFaultBootIntoMotion = pigeon.getStickyFault_BootIntoMotion();
    stickyFaultBootupAccelerometer = pigeon.getStickyFault_BootupAccelerometer();
    stickyFaultBootupGyroscope = pigeon.getStickyFault_BootupGyroscope();

    signals = new BaseStatusSignal[] {
      yaw, pitch, roll, yawVelocity, pitchVelocity, rollVelocity,

      firmwareVersion, hardwareFault, bootDuringEnable, bootIntoMotion, bootupAccelerometer, bootupGyroscope,

      stickyFaultHardware, stickyFaultBootDuringEnable, stickyFaultBootIntoMotion, 
      stickyFaultBootupAccelerometer, stickyFaultBootupGyroscope
    };
  }

  public class Pigeon2IOInputs extends GyroIO.GyroIOInputs {
    public boolean hasCommunication;
		public boolean faultHardware;
		public boolean faultBootDuringEnable;
    public boolean faultBootIntoMotion;
		public boolean faultBootupAccelerometer;
		public boolean faultBootupGyroscope;

    public boolean stickyFaultHardware;
    public boolean stickyFaultBootDuringEnable;
    public boolean stickyFaultBootIntoMotion;
    public boolean stickyFaultBootupAccelerometer;
    public boolean stickyFaultBootupGyroscope;
  }

  @Override
  public void updateInputs(Pigeon2IOInputs inputs) {
    inputs.connected = BaseStatusSignal.refreshAll(signals).isOK();

    inputs.yawPosition = yaw.getValue();
    inputs.yawVelocity = yawVelocity.getValue();
    inputs.pitchPosition = pitch.getValue();
    inputs.pitchVelocity  = pitchVelocity.getValue();
    inputs.rollPosition = roll.getValue();
    inputs.rollVelocity = rollVelocity.getValue();

    inputs.hasCommunication = firmwareVersion.getStatus() == StatusCode.OK;
    inputs.faultHardware = hardwareFault.getValue();
    inputs.faultBootDuringEnable = bootDuringEnable.getValue();
    inputs.faultBootIntoMotion = bootIntoMotion.getValue();
    inputs.faultBootupAccelerometer = bootupAccelerometer.getValue();
    inputs.faultBootupGyroscope = bootupGyroscope.getValue();

    inputs.stickyFaultHardware = stickyFaultHardware.getValue();
    inputs.stickyFaultBootDuringEnable = stickyFaultBootDuringEnable.getValue();
    inputs.stickyFaultBootIntoMotion = stickyFaultBootIntoMotion.getValue();
    inputs.stickyFaultBootupAccelerometer = stickyFaultBootupAccelerometer.getValue();
    inputs.stickyFaultBootupGyroscope = stickyFaultBootupGyroscope.getValue();
  }

  @Override
  public void clearStickyFaults() {
    pigeon.clearStickyFaults();
  }
}
