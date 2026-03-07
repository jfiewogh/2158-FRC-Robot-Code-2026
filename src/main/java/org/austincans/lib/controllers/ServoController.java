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

import edu.wpi.first.wpilibj.Servo;
import org.austincans.frc2026.util.ConfigUtil;

// This class is used to control the servo that is used to rotate the color pie
// The servo is mounted on the robot and is used to rotate the color pie
// @author Austin Cans Robotics Team 2158
public class ServoController {
  private static Servo m_colorServo;
  private static ServoController m_controller = null;

  /**
   * Get instance of ServoController
   *
   * @return ServoController object
   */
  public static ServoController getInstance() {
    if (m_controller == null) m_controller = new ServoController();
    return m_controller;
  }

  // Initialize the servo
  // This method should be called in the robotInit() method of the Robot class
  public static void servoInit() {
    // Initialize the servo here
    try {
      m_colorServo = new Servo(ConfigUtil.getConfigInt("COLOR_SERVO_PORT", 9));
    } catch (edu.wpi.first.hal.util.AllocationException ae) {
      // Let's do nothing
    }
  }

  // Get the servo port
  // This method can be used to get the port number of the servo
  public static int getServoPort() {
    // Get the servo port here
    if (m_colorServo != null) {
      return m_colorServo.getChannel();
    } else {
      System.out.println("Servo error, cannot return port number");
      return -1; // or some other error value
    }
  }

  // Set the servo angle
  // This method should be called to set the angle of the servo
  public static void setServoAngle(double angle) {
    // Set the servo angle here
    if (m_colorServo != null) {
      m_colorServo.setAngle(angle);
    } else {
      System.out.println("Servo error, cannot set angle value");
    }
  }

  // Get the current servo angle
  // This method can be used to read the current angle of the servo
  public static double getServoAngle() {
    // Get the servo angle here
    if (m_colorServo != null) {
      return m_colorServo.getAngle();
    } else {
      System.out.println("Servo error, cannot get angle value");
      return -1; // or some other error value
    }
  }

  // This method can be used to set the servo angle periodically
  // For example, you could use it to update the angle based on some condition
  public static void servoPeriodic() {
    // This method can be used for periodic updates if needed
    // For example, you could read the current angle and update a dashboard
    double currentAngle = getServoAngle();
    System.out.println("Current Servo Angle: " + currentAngle);
  }

  // This method can be used to stop the servo if needed
  // For example, you could stop the servo when the robot is disabled
  public static void servoStop() {
    // Stop the servo here if needed
    if (m_colorServo != null) {
      m_colorServo.setAngle(0); // or some other default value
    } else {
      System.out.println("Servo error, cannot set value to 0");
    }
  }

  // This method can be used to close the servo if needed
  // For example, you could close the servo when the robot is disabled
  public static void servoClose() {
    // Close the servo here if needed
    if (m_colorServo != null) {
      m_colorServo.close();
    } else {
      System.out.println("Servo error, cannot close");
    }
  }

  // This method can be used to set the speed of the servo if needed
  // For example, you could set the speed when the robot is enabled
  public static void servoSetSpeed(double speed) {
    // Set the speed of the servo here if needed
    if (m_colorServo != null) {
      m_colorServo.setSpeed(speed);
    } else {
      System.out.println("Servo error, cannot set speed");
    }
  }

  // This method can be used to get the speed of the servo if needed
  // For example, you could get the speed when the robot is enabled
  public static double servoGetSpeed() {
    // Get the speed of the servo here if needed
    if (m_colorServo != null) {
      return m_colorServo.getSpeed();
    } else {
      System.out.println("Servo error, cannot get speed");
      return -1; // or some other error value
    }
  }

  // This method can be used to set the position of the servo if needed
  // For example, you could set the position when the robot is enabled
  public static void servoSetPosition(double position) {
    // Set the position of the servo here if needed
    if (m_colorServo != null) {
      m_colorServo.setPosition(position);
    } else {
      System.out.println("Servo error, cannot set position");
    }
  }

  // This method can be used to get the position of the servo if needed
  // For example, you could get the position when the robot is enabled
  public static double servoGetPosition() {
    // Get the position of the servo here if needed
    if (m_colorServo != null) {
      return m_colorServo.getPosition();
    } else {
      System.out.println("Servo error, cannot get position");
      return -1; // or some other error value
    }
  }

  public static void setColorPieAngle(String colorPie) {
    // The color pie mounted on servo is a 360 pie divided
    // in 8 45deg slices. Slices are:
    // 0 - Blue
    // 20 - Green
    // 40 - Red
    // 60 - Yellow
    switch (colorPie) {
      case "BLACK":
        double sa = getServoAngle();
        setServoAngle(sa);
        break;
      case "BLUE":
        setServoAngle(0);
        break;
      case "GREEN":
        setServoAngle(20);
        break;
      case "RED":
        setServoAngle(45);
        break;
      case "YELLOW":
        setServoAngle(70);
    }
  }
}
