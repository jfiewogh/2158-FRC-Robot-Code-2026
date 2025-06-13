// Copyright (c) 2025 the ausTIN CANs
// https://github.com/AusTINCANsProgrammingTeam
//
// Use of this source code is governed by a BSD
// license that can be found in the LICENSE file
// at the root directory of this project.

package org.austincans.frc2158.subsystems;

import static edu.wpi.first.units.Units.Volts;

import com.ctre.phoenix6.hardware.TalonFX;
import edu.wpi.first.math.Nat;
import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.controller.LinearQuadraticRegulator;
import edu.wpi.first.math.estimator.KalmanFilter;
import edu.wpi.first.math.numbers.N1;
import edu.wpi.first.math.system.LinearSystem;
import edu.wpi.first.math.system.LinearSystemLoop;
import edu.wpi.first.math.system.plant.LinearSystemId;
import edu.wpi.first.units.measure.Voltage;
import edu.wpi.first.wpilibj.RobotController;
import edu.wpi.first.wpilibj.sysid.SysIdRoutineLog;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine;
import org.littletonrobotics.junction.Logger;

public class Flywheel extends SubsystemBase {
  // Volts per (radian per second)
  private static final double kFlywheelKv = 0.023; // TUNE this

  // Volts per (radian per second squared)
  private static final double kFlywheelKa = 0.001; // TUNE this

  // The plant holds a state-space model of our flywheel. This system has the following properties:
  //
  // States: [velocity], in radians per second.
  // Inputs (what we can "put in"): [voltage], in volts.
  // Outputs (what we can measure): [velocity], in radians per second.
  //
  // The Kv and Ka constants are found using the FRC Characterization toolsuite.
  private final LinearSystem<N1, N1, N1> m_flywheelPlant =
      LinearSystemId.identifyVelocitySystem(kFlywheelKv, kFlywheelKa);

  // The observer fuses our encoder data and voltage inputs to reject noise.
  private final KalmanFilter<N1, N1, N1> m_observer =
      new KalmanFilter<>(
          Nat.N1(),
          Nat.N1(),
          m_flywheelPlant,
          VecBuilder.fill(3.0), // How accurate we think our model is
          VecBuilder.fill(0.01), // How accurate we think our encoder data is
          0.020);

  // A LQR uses feedback to create voltage commands.
  private final LinearQuadraticRegulator<N1, N1, N1> m_controller =
      new LinearQuadraticRegulator<>(
          m_flywheelPlant,
          VecBuilder.fill(8.0), // Velocity error tolerance
          VecBuilder.fill(12.0), // Control effort (voltage) tolerance
          0.020);

  // The state-space loop combines a controller, observer, feedforward and plant for easy control.
  private final LinearSystemLoop<N1, N1, N1> m_loop =
      new LinearSystemLoop<>(m_flywheelPlant, m_controller, m_observer, 12.0, 0.020);

  private TalonFX m_flywheel;

  private final SysIdRoutine m_SysIdRoutine =
      new SysIdRoutine(
          new SysIdRoutine.Config(),
          new SysIdRoutine.Mechanism(this::setVoltage, this::logFlywheel, this));

  /** Creates a new Flywheel. */
  public Flywheel(int port) {
    m_flywheel = new TalonFX(port);
    m_loop.reset(VecBuilder.fill(getEncoderRate()));
  }

  @Override
  public void periodic() {
    // Correct our Kalman filter's state vector estimate with encoder data.
    m_loop.correct(VecBuilder.fill(getEncoderRate()));

    // Update our LQR to generate new voltage commands and use the voltages to predict the next
    // state with out Kalman filter.
    m_loop.predict(0.020);

    // Send the new calculated voltage to the motors.
    // voltage = duty cycle * battery voltage, so
    // duty cycle = voltage / battery voltage
    double nextVoltage = m_loop.getU(0);
    m_flywheel.setVoltage(nextVoltage);
  }

  private double getEncoderRate() {
    double v = m_flywheel.getVelocity().getValueAsDouble();
    return v * 2.0 * Math.PI;
  }

  public void setVelocity(double velocity) {
    m_loop.setNextR(VecBuilder.fill(velocity));
  }

  public Command sysIdQuasistatic(SysIdRoutine.Direction direction) {
    return m_SysIdRoutine.quasistatic(direction);
  }

  public Command sysIdDynamic(SysIdRoutine.Direction direction) {
    return m_SysIdRoutine.dynamic(direction);
  }

  private void setVoltage(Voltage volts) {
    m_flywheel.setVoltage(volts.in(Volts));
  }

  private void logFlywheel(SysIdRoutineLog log) {
    // Unit: Volts
    Logger.recordOutput("Flywheel/Voltage", m_flywheel.get() * RobotController.getBatteryVoltage());
    // Unit: Rotations
    Logger.recordOutput("Flywheel/AnglePosition", m_flywheel.getPosition().getValueAsDouble());
    // Unit: Rotations Per Second
    Logger.recordOutput(
        "Flywheel/AngularVelocity", m_flywheel.getVelocity().getValueAsDouble() * 2.0 * Math.PI);
  }
}
