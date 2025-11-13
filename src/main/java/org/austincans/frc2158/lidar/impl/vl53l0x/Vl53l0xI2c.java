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

package org.austincans.frc2158.lidar.impl.vl53l0x;

import static org.austincans.frc2158.lidar.impl.vl53l0x.Calculations.calcMacroPeriod;
import static org.austincans.frc2158.lidar.impl.vl53l0x.Calculations.decodeVcselPeriod;
import static org.austincans.frc2158.lidar.impl.vl53l0x.Calculations.encodeVcselPeriod;
import static org.austincans.frc2158.lidar.impl.vl53l0x.Vl53l0xReg.ALGO_PHASECAL_CONFIG_TIMEOUT;
import static org.austincans.frc2158.lidar.impl.vl53l0x.Vl53l0xReg.ALGO_PHASECAL_LIM;
import static org.austincans.frc2158.lidar.impl.vl53l0x.Vl53l0xReg.FINAL_RANGE_CONFIG_MIN_COUNT_RATE_RTN_LIMIT;
import static org.austincans.frc2158.lidar.impl.vl53l0x.Vl53l0xReg.FINAL_RANGE_CONFIG_TIMEOUT_MACROP_HI;
import static org.austincans.frc2158.lidar.impl.vl53l0x.Vl53l0xReg.FINAL_RANGE_CONFIG_VALID_PHASE_HIGH;
import static org.austincans.frc2158.lidar.impl.vl53l0x.Vl53l0xReg.FINAL_RANGE_CONFIG_VALID_PHASE_LOW;
import static org.austincans.frc2158.lidar.impl.vl53l0x.Vl53l0xReg.FINAL_RANGE_CONFIG_VCSEL_PERIOD;
import static org.austincans.frc2158.lidar.impl.vl53l0x.Vl53l0xReg.GLOBAL_CONFIG_VCSEL_WIDTH;
import static org.austincans.frc2158.lidar.impl.vl53l0x.Vl53l0xReg.MSRC_CONFIG_TIMEOUT_MACROP;
import static org.austincans.frc2158.lidar.impl.vl53l0x.Vl53l0xReg.OSC_CALIBRATE_VAL;
import static org.austincans.frc2158.lidar.impl.vl53l0x.Vl53l0xReg.PRE_RANGE_CONFIG_TIMEOUT_MACROP_HI;
import static org.austincans.frc2158.lidar.impl.vl53l0x.Vl53l0xReg.PRE_RANGE_CONFIG_VALID_PHASE_HIGH;
import static org.austincans.frc2158.lidar.impl.vl53l0x.Vl53l0xReg.PRE_RANGE_CONFIG_VALID_PHASE_LOW;
import static org.austincans.frc2158.lidar.impl.vl53l0x.Vl53l0xReg.PRE_RANGE_CONFIG_VCSEL_PERIOD;
import static org.austincans.frc2158.lidar.impl.vl53l0x.Vl53l0xReg.RESULT_INTERRUPT_STATUS;
import static org.austincans.frc2158.lidar.impl.vl53l0x.Vl53l0xReg.RESULT_RANGE_STATUS;
import static org.austincans.frc2158.lidar.impl.vl53l0x.Vl53l0xReg.SYSRANGE_START;
import static org.austincans.frc2158.lidar.impl.vl53l0x.Vl53l0xReg.SYSTEM_INTERMEASUREMENT_PERIOD;
import static org.austincans.frc2158.lidar.impl.vl53l0x.Vl53l0xReg.SYSTEM_INTERRUPT_CLEAR;
import static org.austincans.frc2158.lidar.impl.vl53l0x.Vl53l0xReg.SYSTEM_SEQUENCE_CONFIG;
import static org.austincans.frc2158.lidar.util.Preconditions.checkArgument;
import static org.austincans.frc2158.lidar.util.Preconditions.checkState;

import java.util.Objects;
import java.util.Optional;
import java.util.concurrent.TimeUnit;
import org.austincans.frc2158.lidar.api.Error;
import org.austincans.frc2158.lidar.api.Vl53l0x;
import org.austincans.frc2158.lidar.arcompat.PololuI2c;
import org.austincans.frc2158.lidar.arcompat.Port;
import org.austincans.frc2158.lidar.arcompat.Register;
import org.austincans.frc2158.lidar.util.Preconditions;

/**
 * A near-direct port of the <a href="https://github.com/pololu/vl53l0x-arduino">vl53l0x-arduino</a>
 * library.
 */
// The original library is licensed under the terms in LICENSE-vl53l0x.txt
public class Vl53l0xI2c implements Vl53l0x {

  private final PololuI2c i2c;
  private long timeout;
  private long timeoutStart;
  private boolean didTimeout;
  short stopVariable;
  long measurementTimingBudgetMicrosec;

  public Vl53l0xI2c(Port port) {
    this.i2c = PololuI2c.create(port, PololuI2c.Addressing.EIGHT_BIT);
    i2c.setAddress(Vl53l0x.DEFAULT_ADDRESS);
  }

  @Override
  public void close() {
    i2c.close();
  }

  @Override
  public byte getAddress() {
    return i2c.getAddress();
  }

  @Override
  public void setAddress(byte address) {
    if (address == getAddress()) {
      return;
    }
    Vl53l0xReg.I2C_SLAVE_DEVICE_ADDRESS.on(i2c).write((byte) (address & 0x7F));
    i2c.setAddress(address);
  }

  @Override
  public Optional<Error<?>> initialize() {
    return new Vl53l0xInit(this).initialize();
  }

  @Override
  public PololuI2c getI2c() {
    return i2c;
  }

  @Override
  public void setSignalRateLimit(float limitMpcs) {
    checkState(0 <= limitMpcs && limitMpcs <= 511.99, "limitMpcs out of range");

    // Q9.7 fixed point format (9 integer bits, 7 fractional bits)
    FINAL_RANGE_CONFIG_MIN_COUNT_RATE_RTN_LIMIT.on(i2c).write16Bit((int) (limitMpcs * (1 << 7)));
  }

  @Override
  public float getSignalRateLimit() {
    return ((float) FINAL_RANGE_CONFIG_MIN_COUNT_RATE_RTN_LIMIT.on(i2c).read16Bit()) / (1 << 7);
  }

  private static final int StartOverhead = 1910;
  private static final int EndOverhead = 960;
  private static final int MsrcOverhead = 660;
  private static final int TccOverhead = 590;
  private static final int DssOverhead = 690;
  private static final int PreRangeOverhead = 660;
  private static final int FinalRangeOverhead = 550;

  private long initializeBudgetValue(SequenceStepEnables enables, SequenceStepTimeouts timeouts) {
    long microsec = StartOverhead + EndOverhead;

    if (enables.tcc) {
      microsec += (timeouts.msrcDssTccMicrosec + TccOverhead);
    }

    if (enables.dss) {
      microsec += 2 * (timeouts.msrcDssTccMicrosec + DssOverhead);
    } else if (enables.msrc) {
      microsec += (timeouts.msrcDssTccMicrosec + MsrcOverhead);
    }

    if (enables.preRange) {
      microsec += (timeouts.preRangeMicrosec + PreRangeOverhead);
    }
    return microsec;
  }

  private static final long MIN_TIMING_BUDGET = 20000;

  @Override
  public void setMeasurementTimingBudget(long budgetMicrosec) {
    checkArgument(MIN_TIMING_BUDGET <= budgetMicrosec, "budgetMicrosec too small");

    SequenceStepEnables enables = getSequenceStepEnables();
    SequenceStepTimeouts timeouts = getSequenceStepTimeouts(enables);
    long usedBudgetMicrosec = initializeBudgetValue(enables, timeouts);

    if (enables.finalRange) {
      usedBudgetMicrosec += FinalRangeOverhead;

      // "Note that the final range timeout is determined by the timing
      // budget and the sum of all other timeouts within the sequence.
      // If there is no room for the final range timeout, then an error
      // will be set. Otherwise the remaining time will be applied to
      // the final range."

      checkArgument(usedBudgetMicrosec <= budgetMicrosec, "Requested timeout too big");

      long final_range_timeout_us = budgetMicrosec - usedBudgetMicrosec;

      // set_sequence_step_timeout() begin
      // (SequenceStepId == VL53L0X_SEQUENCESTEP_FINAL_RANGE)

      // "For the final range timeout, the pre-range timeout
      //  must be added. To do this both final and pre-range
      //  timeouts must be expressed in macro periods MClks
      //  because they have different vcsel periods."

      long finalRangeTimeoutMclks =
          timeoutMicrosecondsToMclks(final_range_timeout_us, timeouts.finalRangeVcselPeriodPclks);

      if (enables.preRange) {
        finalRangeTimeoutMclks += timeouts.preRangeMclks;
      }

      FINAL_RANGE_CONFIG_TIMEOUT_MACROP_HI
          .on(i2c)
          .write16Bit(encodeTimeout(finalRangeTimeoutMclks));

      // set_sequence_step_timeout() end

      measurementTimingBudgetMicrosec = budgetMicrosec; // store for internal reuse
    }
  }

  @Override
  public long getMeasurementTimingBudget() {
    SequenceStepEnables enables = getSequenceStepEnables();
    SequenceStepTimeouts timeouts = getSequenceStepTimeouts(enables);

    long budgetMicrosec = initializeBudgetValue(enables, timeouts);
    if (enables.finalRange) {
      budgetMicrosec += (timeouts.finalRangeMicrosec + FinalRangeOverhead);
    }

    measurementTimingBudgetMicrosec = budgetMicrosec; // store for internal reuse
    return budgetMicrosec;
  }

  @Override
  public void setVcselPulsePeriod(VcselPeriodType type, short periodPclks) {
    short vcselPeriodReg = encodeVcselPeriod(periodPclks);
    SequenceStepEnables enables = getSequenceStepEnables();
    SequenceStepTimeouts timeouts = getSequenceStepTimeouts(enables);

    switch (type) {
      case VCSEL_PERIOD_PRE_RANGE:
        // "Set phase check limits"
        switch (periodPclks) {
          case 12:
            PRE_RANGE_CONFIG_VALID_PHASE_HIGH.on(i2c).write((short) 0x18);
            break;

          case 14:
            PRE_RANGE_CONFIG_VALID_PHASE_HIGH.on(i2c).write((short) 0x30);
            break;

          case 16:
            PRE_RANGE_CONFIG_VALID_PHASE_HIGH.on(i2c).write((short) 0x40);
            break;

          case 18:
            PRE_RANGE_CONFIG_VALID_PHASE_HIGH.on(i2c).write((short) 0x50);
            break;

          default:
            throw new IllegalArgumentException("Invalid period: " + periodPclks);
        }
        PRE_RANGE_CONFIG_VALID_PHASE_LOW.on(i2c).write((short) 0x08);

        // apply new VCSEL period
        PRE_RANGE_CONFIG_VCSEL_PERIOD.on(i2c).write(vcselPeriodReg);

        // update timeouts

        // set_sequence_step_timeout() begin
        // (SequenceStepId == VL53L0X_SEQUENCESTEP_PRE_RANGE)

        long newPreRangeTimeoutMclks =
            timeoutMicrosecondsToMclks(timeouts.preRangeMicrosec, periodPclks);

        PRE_RANGE_CONFIG_TIMEOUT_MACROP_HI
            .on(i2c)
            .write16Bit(encodeTimeout(newPreRangeTimeoutMclks));

        // set_sequence_step_timeout() end

        // set_sequence_step_timeout() begin
        // (SequenceStepId == VL53L0X_SEQUENCESTEP_MSRC)

        long newMsrcTimeoutMclks =
            timeoutMicrosecondsToMclks(timeouts.msrcDssTccMicrosec, periodPclks);

        MSRC_CONFIG_TIMEOUT_MACROP
            .on(i2c)
            .write((short) ((newMsrcTimeoutMclks > 256) ? 255 : (newMsrcTimeoutMclks - 1)));

        // set_sequence_step_timeout() end
        break;
      case VCSEL_PERIOD_FINAL_RANGE:
        switch (periodPclks) {
          case 8:
            FINAL_RANGE_CONFIG_VALID_PHASE_HIGH.on(i2c).write((short) 0x10);
            FINAL_RANGE_CONFIG_VALID_PHASE_LOW.on(i2c).write((short) 0x08);
            GLOBAL_CONFIG_VCSEL_WIDTH.on(i2c).write((short) 0x02);
            ALGO_PHASECAL_CONFIG_TIMEOUT.on(i2c).write((short) 0x0C);
            i2c.writeReg((short) 0xFF, (short) 0x01);
            ALGO_PHASECAL_LIM.on(i2c).write((short) 0x30);
            i2c.writeReg((short) 0xFF, (short) 0x00);
            break;

          case 10:
            FINAL_RANGE_CONFIG_VALID_PHASE_HIGH.on(i2c).write((short) 0x28);
            FINAL_RANGE_CONFIG_VALID_PHASE_LOW.on(i2c).write((short) 0x08);
            GLOBAL_CONFIG_VCSEL_WIDTH.on(i2c).write((short) 0x03);
            ALGO_PHASECAL_CONFIG_TIMEOUT.on(i2c).write((short) 0x09);
            i2c.writeReg((short) 0xFF, (short) 0x01);
            ALGO_PHASECAL_LIM.on(i2c).write((short) 0x20);
            i2c.writeReg((short) 0xFF, (short) 0x00);
            break;

          case 12:
            FINAL_RANGE_CONFIG_VALID_PHASE_HIGH.on(i2c).write((short) 0x38);
            FINAL_RANGE_CONFIG_VALID_PHASE_LOW.on(i2c).write((short) 0x08);
            GLOBAL_CONFIG_VCSEL_WIDTH.on(i2c).write((short) 0x03);
            ALGO_PHASECAL_CONFIG_TIMEOUT.on(i2c).write((short) 0x08);
            i2c.writeReg((short) 0xFF, (short) 0x01);
            ALGO_PHASECAL_LIM.on(i2c).write((short) 0x20);
            i2c.writeReg((short) 0xFF, (short) 0x00);
            break;

          case 14:
            FINAL_RANGE_CONFIG_VALID_PHASE_HIGH.on(i2c).write((short) 0x48);
            FINAL_RANGE_CONFIG_VALID_PHASE_LOW.on(i2c).write((short) 0x08);
            GLOBAL_CONFIG_VCSEL_WIDTH.on(i2c).write((short) 0x03);
            ALGO_PHASECAL_CONFIG_TIMEOUT.on(i2c).write((short) 0x07);
            i2c.writeReg((short) 0xFF, (short) 0x01);
            ALGO_PHASECAL_LIM.on(i2c).write((short) 0x20);
            i2c.writeReg((short) 0xFF, (short) 0x00);
            break;

          default:
            throw new IllegalArgumentException("Invalid period: " + periodPclks);
        }

        // apply new VCSEL period
        FINAL_RANGE_CONFIG_VCSEL_PERIOD.on(i2c).write(vcselPeriodReg);

        // update timeouts

        // set_sequence_step_timeout() begin
        // (SequenceStepId == VL53L0X_SEQUENCESTEP_FINAL_RANGE)

        // "For the final range timeout, the pre-range timeout
        //  must be added. To do this both final and pre-range
        //  timeouts must be expressed in macro periods MClks
        //  because they have different vcsel periods."

        long timeoutMicrosecondsToMclks =
            timeoutMicrosecondsToMclks(timeouts.finalRangeMicrosec, periodPclks);

        if (enables.preRange) {
          timeoutMicrosecondsToMclks += timeouts.preRangeMclks;
        }

        FINAL_RANGE_CONFIG_TIMEOUT_MACROP_HI
            .on(i2c)
            .write16Bit(encodeTimeout(timeoutMicrosecondsToMclks));

        // set_sequence_step_timeout end
        break;
      default:
        // invalid type
        throw new IllegalArgumentException("Invalid type: " + type);
    }

    // "Finally, the timing budget must be re-applied"

    setMeasurementTimingBudget(measurementTimingBudgetMicrosec);

    // "Perform the phase calibration. This is needed after changing on vcsel period."
    // VL53L0X_perform_phase_calibration() begin

    short sequence_config = SYSTEM_SEQUENCE_CONFIG.on(i2c).read();
    SYSTEM_SEQUENCE_CONFIG.on(i2c).write((short) 0x02);
    performSingleRefCalibration((short) 0x0);
    SYSTEM_SEQUENCE_CONFIG.on(i2c).write(sequence_config);

    // VL53L0X_perform_phase_calibration() end
  }

  @Override
  public short getVcselPulsePeriod(VcselPeriodType type) {
    Register reg;
    switch (type) {
      case VCSEL_PERIOD_PRE_RANGE:
        reg = PRE_RANGE_CONFIG_VCSEL_PERIOD;
        break;
      case VCSEL_PERIOD_FINAL_RANGE:
        reg = FINAL_RANGE_CONFIG_VCSEL_PERIOD;
        break;
      default:
        throw new IllegalStateException("Invalid type: " + type);
    }
    return decodeVcselPeriod(reg.on(i2c).read());
  }

  @Override
  public void startContinuous(long periodMilli) {
    i2c.writeReg((short) 0x80, (short) 0x01);
    i2c.writeReg((short) 0xFF, (short) 0x01);
    i2c.writeReg((short) 0x00, (short) 0x00);
    i2c.writeReg((short) 0x91, stopVariable);
    i2c.writeReg((short) 0x00, (short) 0x01);
    i2c.writeReg((short) 0xFF, (short) 0x00);
    i2c.writeReg((short) 0x80, (short) 0x00);

    if (periodMilli != 0) {
      // continuous timed mode

      // VL53L0X_SetInterMeasurementPeriodMilliSeconds() begin

      int oscCalibrateVal = OSC_CALIBRATE_VAL.on(i2c).read16Bit();

      if (oscCalibrateVal != 0) {
        periodMilli *= oscCalibrateVal;
      }

      SYSTEM_INTERMEASUREMENT_PERIOD.on(i2c).write32Bit(periodMilli);

      // VL53L0X_SetInterMeasurementPeriodMilliSeconds() end

      SYSRANGE_START.on(i2c).write((short) 0x04); // VL53L0X_REG_SYSRANGE_MODE_TIMED
    } else {
      // continuous back-to-back mode
      SYSRANGE_START.on(i2c).write((short) 0x02); // VL53L0X_REG_SYSRANGE_MODE_BACKTOBACK
    }
  }

  @Override
  public void stopContinuous() {
    SYSRANGE_START.on(i2c).write((short) 0x01); // VL53L0X_REG_SYSRANGE_MODE_SINGLESHOT

    i2c.writeReg((short) 0xFF, (short) 0x01);
    i2c.writeReg((short) 0x00, (short) 0x00);
    i2c.writeReg((short) 0x91, (short) 0x00);
    i2c.writeReg((short) 0x00, (short) 0x01);
    i2c.writeReg((short) 0xFF, (short) 0x00);
  }

  @Override
  public boolean dataReady() {
    return (RESULT_INTERRUPT_STATUS.on(i2c).read() & 0x07) == 0;
  }

  @Override
  public int readRangeContinuousMillimeters() {
    checkState(dataReady(), "Data not ready, check dataReady() first.");
    // assumptions: Linearity Corrective Gain is 1000 (default);
    // fractional ranging is not enabled
    int range = i2c.readReg16Bit((short) (RESULT_RANGE_STATUS.address() + 10));

    SYSTEM_INTERRUPT_CLEAR.on(i2c).write((short) 0x01);

    return range;
  }

  @Override
  public int readRangeSingleMillimeters() {
    i2c.writeReg((short) 0x80, (short) 0x01);
    i2c.writeReg((short) 0xFF, (short) 0x01);
    i2c.writeReg((short) 0x00, (short) 0x00);
    i2c.writeReg((short) 0x91, stopVariable);
    i2c.writeReg((short) 0x00, (short) 0x01);
    i2c.writeReg((short) 0xFF, (short) 0x00);
    i2c.writeReg((short) 0x80, (short) 0x00);

    SYSRANGE_START.on(i2c).write((short) 0x01);

    // "Wait until start bit has been cleared"
    startTimeout();
    while ((SYSRANGE_START.on(i2c).read() & 0x01) != 0) {
      if (currentlyTimedOut()) {
        setTimeoutFlag();
        return 65535;
      }
    }
    startTimeout();
    while (!dataReady()) {
      if (currentlyTimedOut()) {
        setTimeoutFlag();
        return 65535;
      }
    }

    return readRangeContinuousMillimeters();
  }

  @Override
  public void setTimeout(long timeout, TimeUnit unit) {
    Preconditions.checkArgument(timeout >= 0, "Timeout must be positive");
    Objects.requireNonNull(unit, "unit");
    this.timeout = unit.toNanos(timeout);
  }

  @Override
  public long getTimeout(TimeUnit unit) {
    return unit.convert(timeout, TimeUnit.NANOSECONDS);
  }

  @Override
  public boolean timeoutOccurred() {
    boolean hasTimedOut = didTimeout;
    didTimeout = false;
    return hasTimedOut;
  }

  Optional<SpadInfo> getSpadInfo() {
    short tmp;

    i2c.writeReg((short) 0x80, (short) 0x01);
    i2c.writeReg((short) 0xFF, (short) 0x01);
    i2c.writeReg((short) 0x00, (short) 0x00);

    i2c.writeReg((short) 0xFF, (short) 0x06);
    i2c.writeReg((short) 0x83, (short) (i2c.readReg((short) 0x83) | 0x04));
    i2c.writeReg((short) 0xFF, (short) 0x07);
    i2c.writeReg((short) 0x81, (short) 0x01);

    i2c.writeReg((short) 0x80, (short) 0x01);

    i2c.writeReg((short) 0x94, (short) 0x6b);
    i2c.writeReg((short) 0x83, (short) 0x00);
    startTimeout();
    while (i2c.readReg((short) 0x83) == 0x00) {
      if (currentlyTimedOut()) {
        setTimeoutFlag();
        return Optional.empty();
      }
    }
    i2c.writeReg((short) 0x83, (short) 0x01);
    tmp = i2c.readReg((short) 0x92);

    SpadInfo spadInfo = new SpadInfo((short) (tmp & 0x7f), ((tmp >> 7) & 0x01) != 0);

    i2c.writeReg((short) 0x81, (short) 0x00);
    i2c.writeReg((short) 0xFF, (short) 0x06);
    i2c.writeReg((short) 0x83, (short) (i2c.readReg((short) 0x83) & ~0x04));
    i2c.writeReg((short) 0xFF, (short) 0x01);
    i2c.writeReg((short) 0x00, (short) 0x01);

    i2c.writeReg((short) 0xFF, (short) 0x00);
    i2c.writeReg((short) 0x80, (short) 0x00);

    return Optional.of(spadInfo);
  }

  SequenceStepEnables getSequenceStepEnables() {
    short sequence_config = SYSTEM_SEQUENCE_CONFIG.on(i2c).read();

    return new SequenceStepEnables(
        ((sequence_config >> 4) & 0x1) != 0,
        ((sequence_config >> 3) & 0x1) != 0,
        ((sequence_config >> 2) & 0x1) != 0,
        ((sequence_config >> 6) & 0x1) != 0,
        ((sequence_config >> 7) & 0x1) != 0);
  }

  SequenceStepTimeouts getSequenceStepTimeouts(SequenceStepEnables enables) {
    short preRangeVcselPeriodPclks = getVcselPulsePeriod(VcselPeriodType.VCSEL_PERIOD_PRE_RANGE);

    int msrcDssTccMclks = MSRC_CONFIG_TIMEOUT_MACROP.on(i2c).read() + 1;
    long msrcDssTccMicroseconds =
        timeoutMclksToMicroseconds(msrcDssTccMclks, preRangeVcselPeriodPclks);

    int preRangeMclks = decodeTimeout(PRE_RANGE_CONFIG_TIMEOUT_MACROP_HI.on(i2c).read16Bit());
    long preRangeMicroseconds = timeoutMclksToMicroseconds(preRangeMclks, preRangeVcselPeriodPclks);

    short finalRangeVcselPeriodPclks =
        getVcselPulsePeriod(VcselPeriodType.VCSEL_PERIOD_FINAL_RANGE);

    int finalRangeMclks = decodeTimeout(FINAL_RANGE_CONFIG_TIMEOUT_MACROP_HI.on(i2c).read16Bit());

    if (enables.preRange) {
      finalRangeMclks -= preRangeMclks;
    }

    long finalRangeMicroseconds =
        timeoutMclksToMicroseconds(finalRangeMclks, finalRangeVcselPeriodPclks);

    return new SequenceStepTimeouts(
        preRangeVcselPeriodPclks,
        finalRangeVcselPeriodPclks,
        msrcDssTccMclks,
        preRangeMclks,
        finalRangeMclks,
        msrcDssTccMicroseconds,
        preRangeMicroseconds,
        finalRangeMicroseconds);
  }

  boolean performSingleRefCalibration(short vhvInitByte) {
    SYSRANGE_START
        .on(i2c)
        .write((short) (0x01 | vhvInitByte)); // VL53L0X_REG_SYSRANGE_MODE_START_STOP

    startTimeout();
    while ((RESULT_INTERRUPT_STATUS.on(i2c).read() & 0x07) == 0) {
      if (currentlyTimedOut()) {
        setTimeoutFlag();
        return false;
      }
    }

    SYSTEM_INTERRUPT_CLEAR.on(i2c).write((short) 0x01);

    SYSRANGE_START.on(i2c).write((short) 0x00);

    return true;
  }

  static int decodeTimeout(int value) {
    return ((value & 0x00FF) << ((value & 0xFF00) >> 8)) + 1;
  }

  static int encodeTimeout(long timeoutMclks) {
    long ls_byte = 0;
    int ms_byte = 0;

    if (timeoutMclks > 0) {
      ls_byte = timeoutMclks - 1;

      while ((ls_byte & 0xFFFFFF00) > 0) {
        ls_byte >>= 1;
        ms_byte++;
      }

      return (int) ((ms_byte << 8) | (ls_byte & 0xFF));
    } else {
      return 0;
    }
  }

  static long timeoutMclksToMicroseconds(int timeoutPeriodMclks, short vcselPeriodPclks) {
    long macroPeriodNs = calcMacroPeriod(vcselPeriodPclks);

    return ((timeoutPeriodMclks * macroPeriodNs) + (macroPeriodNs / 2)) / 1000;
  }

  static long timeoutMicrosecondsToMclks(long timeoutPeriodUs, int vcselPeriodPclks) {
    long macroPeriodNs = calcMacroPeriod(vcselPeriodPclks);

    return (((timeoutPeriodUs * 1000) + (macroPeriodNs / 2)) / macroPeriodNs);
  }

  // internal functions:

  void startTimeout() {
    timeoutStart = System.nanoTime();
  }

  boolean currentlyTimedOut() {
    if (timeout <= 0) {
      return false;
    }

    return timeoutStart + timeout < System.nanoTime();
  }

  void setTimeoutFlag() {
    didTimeout = true;
  }
}
