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
