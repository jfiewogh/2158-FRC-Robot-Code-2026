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

package org.austincans.lib.util;

import java.io.FileNotFoundException;
import java.io.PrintWriter;
import java.lang.reflect.Field;
import java.util.concurrent.ConcurrentLinkedDeque;

/**
 * Writes data to a CSV file
 *
 * @author Austin Cans Robotics Team 2158
 */
public class ReflectingCSVWriter<T> {
  private ConcurrentLinkedDeque<String> mLinesToWrite = new ConcurrentLinkedDeque<>();
  private PrintWriter mOutput = null;
  private Field[] mFields;

  public ReflectingCSVWriter(String fileName, Class<T> typeClass) {
    mFields = typeClass.getFields();
    try {
      mOutput = new PrintWriter(fileName);
    } catch (FileNotFoundException e) {
      e.printStackTrace();
    }
    // Write field names.
    StringBuilder line = new StringBuilder();
    for (Field field : mFields) {
      if (line.length() != 0) {
        line.append(", ");
      }
      line.append(field.getName());
    }
    writeLine(line.toString());
  }

  public void add(T value) {
    StringBuilder line = new StringBuilder();
    for (Field field : mFields) {
      if (line.length() != 0) {
        line.append(", ");
      }
      try {
        if (CSVWritable.class.isAssignableFrom(field.getType())) {
          line.append(((CSVWritable) field.get(value)).toCSV());
        } else {
          line.append(field.get(value).toString());
        }
      } catch (IllegalArgumentException | IllegalAccessException e) {
        e.printStackTrace();
      }
    }
    mLinesToWrite.add(line.toString());
  }

  protected synchronized void writeLine(String line) {
    if (mOutput != null) {
      mOutput.println(line);
    }
  }

  // Call this periodically from any thread to write to disk.
  public void write() {
    while (true) {
      String val = mLinesToWrite.pollFirst();
      if (val == null) {
        break;
      }
      writeLine(val);
    }
  }

  public synchronized void flush() {
    if (mOutput != null) {
      write();
      mOutput.flush();
    }
  }
}
