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
// Updated on Thursday November 13, 2025 at 14:16:40 CST

package org.austincans.frc2158.lidar.impl.vl53l1x;

import java.util.Optional;
import java.util.concurrent.TimeUnit;
import org.austincans.frc2158.lidar.api.Error;
import org.austincans.frc2158.lidar.arcompat.PololuI2c;
import org.austincans.frc2158.lidar.arcompat.Register;
import org.austincans.frc2158.lidar.impl.errors.IncorrectModelId;
import org.austincans.frc2158.lidar.impl.errors.Timeout;
import org.austincans.frc2158.lidar.util.SleepEasy;

/** Companion class for initializing {@link Vl53l1xI2c}. */
class Vl53l1xInit {
  private static final int MODEL_ID = 0xEACC;
  private final Vl53l1xI2c target;
  private final PololuI2c i2c;

  Vl53l1xInit(Vl53l1xI2c target) {
    this.target = target;
    this.i2c = target.getI2c();
  }

  Optional<Error<?>> initialize() {
    int modelId = Vl53l1xReg.IDENTIFICATION__MODEL_ID.on(i2c).read16Bit();
    if (modelId != MODEL_ID) {
      return Optional.of(IncorrectModelId.of("VL53L1X", MODEL_ID, modelId));
    }

    Vl53l1xReg.SOFT_RESET.on(i2c).write((byte) 0);
    // wait for reset...
    SleepEasy.forUnit(100, TimeUnit.MICROSECONDS);
    Vl53l1xReg.SOFT_RESET.on(i2c).write((byte) 1);

    SleepEasy.forUnit(1, TimeUnit.SECONDS);

    if (!awaitSystemBooted()) {
      return Optional.of(Timeout.waitingFor("system boot"));
    }

    // set io_2v8
    Register.Bound padI2cHvExtsupConfig = Vl53l1xReg.PAD_I2C_HV__EXTSUP_CONFIG.on(i2c);
    padI2cHvExtsupConfig.write((short) (padI2cHvExtsupConfig.read() | 0x01));

    target.fastOscFreq(Vl53l1xReg.OSC_MEASURED__FAST_OSC__FREQUENCY.on(i2c).read16Bit());
    target.oscCalibrateVal(Vl53l1xReg.RESULT__OSC_CALIBRATE_VAL.on(i2c).read16Bit());

    writeConfig();

    target.setDistanceMode(DistanceMode.LONG);
    target.setMeasurementTimingBudget(50_000);

    Vl53l1xReg.ALGO__PART_TO_PART_RANGE_OFFSET_MM
        .on(i2c)
        .write16Bit((short) (Vl53l1xReg.MM_CONFIG__OUTER_OFFSET_MM.on(i2c).read16Bit() * 4));

    return Optional.empty();
  }

  private boolean awaitSystemBooted() {
    target.startTimeout();
    Register.Bound sysStatus = Vl53l1xReg.FIRMWARE__SYSTEM_STATUS.on(i2c);
    while (true) {
      if ((sysStatus.read() & 0x01) != 0 && i2c.wasLastOpSuccessful()) {
        return true;
      }

      if (target.currentlyTimedOut()) {
        target.setTimeoutFlag();
        return false;
      }
    }
  }

  private void writeConfig() {
    Vl53l1xReg.DSS_CONFIG__TARGET_TOTAL_RATE_MCPS.on(i2c).write16Bit(Vl53l1xI2c.TARGET_RATE);
    Vl53l1xReg.GPIO__TIO_HV_STATUS.on(i2c).write((short) 0x02);
    Vl53l1xReg.SIGMA_ESTIMATOR__EFFECTIVE_PULSE_WIDTH_NS.on(i2c).write((short) 8);
    Vl53l1xReg.SIGMA_ESTIMATOR__EFFECTIVE_AMBIENT_WIDTH_NS.on(i2c).write((short) 16);
    Vl53l1xReg.ALGO__CROSSTALK_COMPENSATION_VALID_HEIGHT_MM.on(i2c).write((short) 0x01);
    Vl53l1xReg.ALGO__RANGE_IGNORE_VALID_HEIGHT_MM.on(i2c).write((short) 0xFF);
    Vl53l1xReg.ALGO__RANGE_MIN_CLIP.on(i2c).write((short) 0);
    Vl53l1xReg.ALGO__CONSISTENCY_CHECK__TOLERANCE.on(i2c).write((short) 2);

    Vl53l1xReg.SYSTEM__THRESH_RATE_HIGH.on(i2c).write16Bit(0);
    Vl53l1xReg.SYSTEM__THRESH_RATE_LOW.on(i2c).write16Bit(0);
    Vl53l1xReg.DSS_CONFIG__APERTURE_ATTENUATION.on(i2c).write((short) 0x38);

    Vl53l1xReg.RANGE_CONFIG__SIGMA_THRESH.on(i2c).write16Bit(360);
    Vl53l1xReg.RANGE_CONFIG__MIN_COUNT_RATE_RTN_LIMIT_MCPS.on(i2c).write16Bit((short) 192);

    Vl53l1xReg.SYSTEM__GROUPED_PARAMETER_HOLD_0.on(i2c).write((short) 1);
    Vl53l1xReg.SYSTEM__GROUPED_PARAMETER_HOLD_1.on(i2c).write((short) 1);
    Vl53l1xReg.SD_CONFIG__QUANTIFIER.on(i2c).write((short) 2);

    Vl53l1xReg.SYSTEM__GROUPED_PARAMETER_HOLD.on(i2c).write((short) 0);
    Vl53l1xReg.SYSTEM__SEED_CONFIG.on(i2c).write((short) 1);

    Vl53l1xReg.SYSTEM__SEQUENCE_CONFIG.on(i2c).write((short) 0x8B);
    Vl53l1xReg.DSS_CONFIG__MANUAL_EFFECTIVE_SPADS_SELECT.on(i2c).write16Bit(200 << 8);
    Vl53l1xReg.DSS_CONFIG__ROI_MODE_CONTROL.on(i2c).write((short) 2);
  }
}
