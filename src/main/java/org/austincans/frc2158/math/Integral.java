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

package org.austincans.frc2158.math;

public class Integral {
  SingleVariableEquation method;
  private double step;

  /**
   * Constructors
   *
   * @param method
   * @param step
   */
  public Integral(SingleVariableEquation method, double step) {
    this.method = method;
    this.step = step;
  }

  public Integral(SingleVariableEquation method) {
    this(method, 0.0001);
  }

  /**
   * Calculate the integration
   *
   * @param startPoint
   * @param endPoint
   * @return the sum as a double
   */
  public double calculate(double startPoint, double endPoint) {
    double sum = 0;
    for (double x = startPoint; x < endPoint; x += step) {
      sum += method.evaluate(x) * step;
    }

    return sum;
  }

  /**
   * See
   * https://math.libretexts.org/Bookshelves/Calculus/Calculus_3e_(Apex)/05%3A_Integration/5.03%3A_Riemann_Sums
   *
   * @param xPoints
   * @return the sum as a double
   */
  public double rightRiemannSum(double[] xPoints) {
    double sum = 0;
    for (int i = 0; i < xPoints.length - 1; i++) {
      sum += method.evaluate(xPoints[i]) * (xPoints[i + 1] - xPoints[i]);
    }

    return sum;
  }

  /**
   * See
   * https://math.libretexts.org/Bookshelves/Calculus/Calculus_3e_(Apex)/05%3A_Integration/5.03%3A_Riemann_Sums
   *
   * @param xPoints
   * @return the sum as a double
   */
  public double leftRiemannSum(double[] xPoints) {
    double sum = 0;
    for (int i = 1; i < xPoints.length; i++) {
      sum += method.evaluate(xPoints[i]) * (xPoints[i] - xPoints[i - 1]);
    }

    return sum;
  }
}
