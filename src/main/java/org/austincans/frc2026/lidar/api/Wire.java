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

package org.austincans.frc2026.lidar.api;

import edu.wpi.first.hal.I2CJNI;
import java.nio.ByteBuffer;
import java.nio.ByteOrder;
import java.util.IllegalFormatException;
import java.util.Objects;
import org.austincans.frc2026.lidar.arcompat.Port;
import org.austincans.frc2026.lidar.util.Preconditions;

/**
 * Drop-in replacement for Arduino's Wire class. Only works for master mode.
 *
 * <p>Avoids the higher level I2C since Wire is low level too.
 */
public class Wire implements AutoCloseable {

  private final Port port;

  public Wire(Port port) {
    this.port = Objects.requireNonNull(port, "port");
  }

  public Port getPort() {
    return port;
  }

  // Wire works by building the arrays using begin/end transmission wrappers.
  private static final int BUFFER_LENGTH = 32;
  private boolean open = false;
  private boolean transmitting = false;
  private ByteBuffer rxBuffer =
      ByteBuffer.allocateDirect(BUFFER_LENGTH).order(ByteOrder.BIG_ENDIAN);
  private ByteBuffer txBuffer =
      ByteBuffer.allocateDirect(BUFFER_LENGTH).order(ByteOrder.BIG_ENDIAN);
  private byte txAddress = -1;

  /**
   * Initialize this {@link Wire} object. Essentially just calls {@link I2CJNI#i2CInitialize(int)}.
   */
  public void begin() {
    resetRxBuffer();

    resetTxBuffer();

    I2CJNI.i2CInitialize(port.value());
    open = true;
  }

  @Override
  public void close() {
    I2CJNI.i2CClose(port.value());
    open = false;
  }

  private void resetRxBuffer() {
    rxBuffer.clear();
  }

  private void resetTxBuffer() {
    txBuffer.clear();
  }

  private void checkOpen() {
    Preconditions.checkState(open, "Not open, please call begin() first");
  }

  /**
   * Start buffering for a transmission to {@code address}.
   *
   * @param address the address to send to
   */
  public void beginTransmission(byte address) {
    Preconditions.checkArgument(address >= 0, "Address may not be null");
    checkOpen();
    transmitting = true;
    txAddress = address;
    resetTxBuffer();
  }

  private void checkTransmitting() {
    Preconditions.checkState(transmitting, "Not transmitting, please use beginTransmission()");
  }

  /**
   * End the current transmission and write it.
   *
   * @return {@code true} if successfully transmitted
   */
  public boolean endTransmission() {
    checkOpen();
    checkTransmitting();
    boolean success =
        I2CJNI.i2CWrite(port.value(), txAddress, txBuffer, (byte) txBuffer.position()) >= 0;

    resetTxBuffer();
    transmitting = false;

    return success;
  }

  private void checkWriteable(int amt) {
    Preconditions.checkState(txBuffer.remaining() >= amt, "No more room in the buffer");
  }

  public void write(short data) {
    checkOpen();
    checkTransmitting();
    checkWriteable(Byte.BYTES);
    txBuffer.put((byte) (data & 0xFF));
  }

  public void writeShort(int data) {
    checkOpen();
    checkTransmitting();
    checkWriteable(Short.BYTES);
    txBuffer.put((byte) (data >>> 8 & 0xFF));
    txBuffer.put((byte) (data & 0xFF));
  }

  public void writeInt(long data) {
    checkOpen();
    checkTransmitting();
    checkWriteable(Integer.BYTES);
    txBuffer.put((byte) (data >>> 24 & 0xFF));
    txBuffer.put((byte) (data >>> 16 & 0xFF));
    txBuffer.put((byte) (data >>> 8 & 0xFF));
    txBuffer.put((byte) (data & 0xFF));
  }

  public void write(byte[] data) {
    checkOpen();
    checkTransmitting();
    Preconditions.checkState(txBuffer.remaining() >= data.length, "No more room in the buffer");
    txBuffer.put(data);
  }

  public void requestFrom(byte address, byte amount, boolean continueTransmission) {
    Preconditions.checkArgument(
        0 <= amount && amount <= BUFFER_LENGTH, "amount must be within buffer bounds");
    checkOpen();
    if (continueTransmission) {
      checkTransmitting();
      I2CJNI.i2CTransaction(
          port.value(), address, txBuffer, (byte) txBuffer.position(), rxBuffer, amount);
      resetTxBuffer();
      transmitting = false;
    } else {
      I2CJNI.i2CRead(port.value(), address, rxBuffer, amount);
    }
    rxBuffer.position(0).limit(amount);
  }

  private void checkReadable(int amt) {
    Preconditions.checkState(rxBuffer.remaining() >= amt, "No more data available");
  }

  public short read() {
    checkOpen();
    checkReadable(Byte.BYTES);
    return (short) Byte.toUnsignedInt(rxBuffer.get());
  }

  public int readShort() {
    checkOpen();
    checkReadable(Short.BYTES);
    return Short.toUnsignedInt(rxBuffer.getShort());
  }

  public long readInt() {
    checkOpen();
    checkReadable(Integer.BYTES);
    return Integer.toUnsignedLong(rxBuffer.getInt());
  }

  public void read(byte[] out) {
    checkOpen();
    checkReadable(out.length);
    rxBuffer.get(out);
  }

  public void scanI2cBus() {
    boolean error;
    int loopAddress;
    byte address = 1;
    int nDevices = 0;
    begin();
    checkOpen();
    for (loopAddress = 1; loopAddress < 127; loopAddress++) {
      // The I2C scanner uses the return value of
      // the Write.endTransmission to see if
      // a device did acknowledge to the address.
      beginTransmission(address);
      error = endTransmission();
      if (error) {
        System.out.print("I2C device found at address ");
        try {
          System.out.print(String.format("0x%02X", loopAddress));
          System.out.println(" !");
          nDevices++;
        } catch (IllegalFormatException ife) {
          System.err.println("Format conversion exception: " + ife.getLocalizedMessage());
        }
      }
      address++;
    }
    if (nDevices == 0) {
      System.out.println("No I2C devices found\n");
    } else {
      System.out.println("I2C bus scan completed\n");
    }
    close();
  }
}
