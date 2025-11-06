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
// Updated on Wednesday November 05, 2025 at 18:44:58 CST

package org.austincans.frc2158.controllers;

import com.revrobotics.ColorMatch;
import com.revrobotics.ColorMatchResult;
import com.revrobotics.ColorSensorV3;
import edu.wpi.first.wpilibj.I2C;
import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj.util.Color;
import org.austincans.frc2158.util.LoggedTracer;

/**
 * This is a simple example to show how the REV Color Sensor V3 can be used to detect pre-configured
 * colors.
 *
 * @author Austin Cans Robotics Team 2158
 */
public class ColorSensorController extends TimedRobot {
  /**
   * The I2C port the color sensor is connected to. The REV Color Sensor V3 uses the onboard I2C
   * port on the RoboRIO.
   *
   * <p>Note: The I2C port can be changed to any of the other ports on the RoboRIO if needed.
   */
  private static final I2C.Port i2cPort = I2C.Port.kOnboard;

  /**
   * Note: Any example colors should be calibrated as the user needs, these are here as a basic
   * example.
   */
  private static final Color kBlueTarget = new Color(0.143, 0.427, 0.429);

  private static final Color kGreenTarget = new Color(0.197, 0.561, 0.240);
  private static final Color kRedTarget = new Color(0.561, 0.232, 0.114);
  private static final Color kYellowTarget = new Color(0.361, 0.524, 0.113);

  /**
   * A Rev Color Sensor V3 object is constructed with an I2C port as a parameter. The device will be
   * automatically initialized with default parameters.
   */
  private static final ColorSensorV3 m_colorSensor = new ColorSensorV3(i2cPort);

  /**
   * A Rev Color Match object is used to register and detect known colors. This can be calibrated
   * ahead of time or during operation.
   *
   * <p>This object uses a simple euclidian distance to estimate the closest match with given
   * confidence range.
   */
  private static final ColorMatch m_colorMatcher = new ColorMatch();

  public static void sensorInit() {
    m_colorMatcher.addColorMatch(kBlueTarget);
    m_colorMatcher.addColorMatch(kGreenTarget);
    m_colorMatcher.addColorMatch(kRedTarget);
    m_colorMatcher.addColorMatch(kYellowTarget);
  }

  /**
   * Find a color match.
   *
   * @return the integer returned will be equal to the position in the color array passes to the
   *     constructor. Will return -1 if the confidence in not high enough.
   */
  public int findColorMatch() {
    int position = -1;
    Color[] m_Colors = {kBlueTarget, kGreenTarget, kRedTarget, kYellowTarget};
    Color detected = m_colorSensor.getColor();

    for (int i = 0; i < m_Colors.length; i++) {
      ColorMatchResult match = m_colorMatcher.matchClosestColor(detected);
      if (match.color == m_Colors[i]) {
        if (match.confidence >= getConfidence()) position = i;
      }
    }

    if (position == -1) LoggedTracer.record("WARNING: No Color Found in CAN Utility Color Sensor");

    return position;
  }

  /**
   * This function is called periodically during operator control and is used to detect the color of
   * the object in front of the sensor.
   */
  public static void sensorPeriodic() {

    /** Open Smart Dashboard or Shuffleboard to see the color detected by the sensor. */
    SmartDashboard.putNumber("Red", getDetectedRed());
    SmartDashboard.putNumber("Green", getDetectedGreen());
    SmartDashboard.putNumber("Blue", getDetectedBlue());
    SmartDashboard.putNumber("Confidence", getConfidence());
    SmartDashboard.putString("Detected Color", getDetectedColor());
    SmartDashboard.putNumber("IR", getInfraRed());

    /**
     * Proximity can be used to roughly approximate the distance of an object or provide a threshold
     * for when an object is close enough to provide accurate color values.
     */
    SmartDashboard.putNumber("Proximity", getProximity());
  }

  public static double getDetectedRed() {
    Color detectedColor = m_colorSensor.getColor();
    return detectedColor.red;
  }

  public static double getDetectedGreen() {
    Color detectedColor = m_colorSensor.getColor();
    return detectedColor.green;
  }

  public static double getDetectedBlue() {
    Color detectedColor = m_colorSensor.getColor();
    return detectedColor.blue;
  }

  public static double getConfidence() {
    Color detectedColor = m_colorSensor.getColor();
    ColorMatchResult match = m_colorMatcher.matchClosestColor(detectedColor);
    return match.confidence;
  }

  public static String getDetectedColor() {
    Color detectedColor = m_colorSensor.getColor();
    String colorString;
    ColorMatchResult match = m_colorMatcher.matchClosestColor(detectedColor);

    if (match.color == kBlueTarget) {
      colorString = "Blue";
    } else if (match.color == kRedTarget) {
      colorString = "Red";
    } else if (match.color == kGreenTarget) {
      colorString = "Green";
    } else if (match.color == kYellowTarget) {
      colorString = "Yellow";
    } else {
      colorString = "Unknown";
    }
    return colorString;
  }

  public static double getInfraRed() {
    /** The sensor returns a raw IR value of the infrared light detected. */
    double IR = m_colorSensor.getIR();
    return IR;
  }

  /**
   * In addition to RGB IR values, the color sensor can also return an infrared proximity value. The
   * chip contains an IR led which will emit IR pulses and measure the intensity of the return. When
   * an object is close the value of the proximity will be large (max 2047 with default settings)
   * and will approach zero when the object is far away. Proximity can be used to roughly
   * approximate the distance of an object or provide a threshold for when an object is close enough
   * to provide accurate color values.
   */
  public static int getProximity() {
    int proximity = m_colorSensor.getProximity();
    return proximity;
  }
}
