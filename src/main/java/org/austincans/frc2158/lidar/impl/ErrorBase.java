// Copyright (c) 2025 the ausTIN CANs
// https://github.com/AusTINCANsProgrammingTeam
//
// Use of this source code is governed by a BSD
// license that can be found in the LICENSE file
// at the root directory of this project.

package org.austincans.frc2158.lidar.impl;

import org.austincans.frc2158.lidar.api.Error;

public abstract class ErrorBase<T> implements Error<T> {

  private final String code;
  private final T data;

  protected ErrorBase(String code, T data) {
    this.code = code;
    this.data = data;
  }

  @Override
  public final String code() {
    return code;
  }

  @Override
  public T data() {
    return data;
  }

  protected String formatMessage() {
    return data.toString();
  }

  @Override
  public String toString() {
    return "[" + code + "] " + formatMessage();
  }
}
