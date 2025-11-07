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

package org.austincans.frc2158.subsystems;

import static edu.wpi.first.units.Units.Volts;

import com.ctre.phoenix6.hardware.TalonFX;
import com.revrobotics.spark.SparkFlex;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.SparkMax;
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

/*
 * In statistics and control theory, Kalman filtering (also known as linear quadratic estimation)
 * is an algorithm that uses a series of measurements observed over time, including statistical
 * noise and other inaccuracies, to produce estimates of unknown variables that tend to be more
 * accurate than those based on a single measurement, by estimating a joint probability
 * distribution over the variables for each time-step.
 * The filter is constructed as a mean squared error minimiser, but an alternative derivation of
 * the filter is also provided showing how the filter relates to maximum likelihood statistics.
 * https://docs.wpilib.org/en/stable/docs/software/advanced-controls/state-space/state-space-observers.html#kalman-filters
 */
public class Flywheel extends SubsystemBase {
    // Can be defined in config instead
    public boolean isSparkMax = false;
    public boolean isSparkFlex = true;
    public boolean isTalonFX = false;
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
    // Use Nat to express a natural number as a java class
    private final KalmanFilter<N1, N1, N1> m_observer = new KalmanFilter<>(
            Nat.N1(),
            Nat.N1(),
            m_flywheelPlant,
            VecBuilder.fill(3.0), // How accurate we think our model is
            VecBuilder.fill(0.01), // How accurate we think our encoder data is
            0.020);

    // A LQR uses feedback to create voltage commands.
    private final LinearQuadraticRegulator<N1, N1, N1> m_controller = new LinearQuadraticRegulator<>(
            m_flywheelPlant,
            VecBuilder.fill(8.0), // Velocity error tolerance
            VecBuilder.fill(12.0), // Control effort (voltage) tolerance
            0.020);

    // The state-space loop combines a controller, observer, feedforward and plant for easy control.
    private final LinearSystemLoop<N1, N1, N1> m_loop =
            new LinearSystemLoop<>(m_flywheelPlant, m_controller, m_observer, 12.0, 0.020);

    // CTRE
    private TalonFX m_flywheel_tfx;
    // REV
    private SparkFlex m_flywheel_sf;
    private SparkMax m_flywheel_sm;

    // CAN ID port number
    private int port;

    private final SysIdRoutine m_SysIdRoutine = new SysIdRoutine(
            new SysIdRoutine.Config(), new SysIdRoutine.Mechanism(this::setVoltage, this::logFlywheel, this));

    /** Creates a new Flywheel. */
    public Flywheel(int port) {
        this.port = port;
        m_loop.reset(VecBuilder.fill(getEncoderRate()));
    }

    /** Get the object represented by motor class choosen */
    public Object getMotor() {
        int port = this.port;
        if (isTalonFX) {
            m_flywheel_tfx = new TalonFX(port);
            return m_flywheel_tfx;
        }
        if (isSparkFlex) {
            m_flywheel_sf = new SparkFlex(port, MotorType.kBrushless);
            return m_flywheel_sf;
        } else {
            // SparkMax
            m_flywheel_sm = new SparkMax(port, MotorType.kBrushless);
            return m_flywheel_sm;
        }
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
        // TalonFX
        if (getMotor() instanceof TalonFX) {
            TalonFX m_flywheel = (TalonFX) getMotor();
            m_flywheel.setVoltage(nextVoltage);
        } else {
            // SparkFlex
            if (getMotor() instanceof SparkFlex) {
                SparkFlex m_flywheel = (SparkFlex) getMotor();
                m_flywheel.setVoltage(nextVoltage);
            } else {
                // SparkMax
                SparkMax m_flywheel = (SparkMax) getMotor();
                m_flywheel.setVoltage(nextVoltage);
            }
        }
    }

    private double getEncoderRate() {
        double v = 0;
        // TalonFX
        if (getMotor() instanceof TalonFX) {
            TalonFX m_flywheel = (TalonFX) getMotor();
            v = m_flywheel.getVelocity().getValueAsDouble();
        } else {
            // SparkFlex
            if (getMotor() instanceof SparkFlex) {
                SparkFlex m_flywheel = (SparkFlex) getMotor();
                v = m_flywheel.getEncoder().getVelocity();
            } else {
                // SparkMax
                SparkMax m_flywheel = (SparkMax) getMotor();
                v = m_flywheel.getEncoder().getVelocity();
            }
        }
        return v * 2.0 * Math.PI;
    }

    /* Set Velocity */
    public void setVelocity(double velocity) {
        m_loop.setNextR(VecBuilder.fill(velocity));
    }

    /*
    SysId
    https://docs.wpilib.org/en/stable/docs/software/advanced-controls/system-identification/introduction.html
    Set Quasistatic
    */
    public Command sysIdQuasistatic(SysIdRoutine.Direction direction) {
        return m_SysIdRoutine.quasistatic(direction);
    }

    public Command sysIdDynamic(SysIdRoutine.Direction direction) {
        return m_SysIdRoutine.dynamic(direction);
    }

    /* Set Motor voltage */
    private void setVoltage(Voltage volts) {
        // TalonFX
        if (getMotor() instanceof TalonFX) {
            TalonFX m_flywheel = (TalonFX) getMotor();
            m_flywheel.setVoltage(volts.in(Volts));
        } else {
            // SparkFlex
            if (getMotor() instanceof SparkFlex) {
                SparkFlex m_flywheel = (SparkFlex) getMotor();
                m_flywheel.setVoltage(volts.in(Volts));
            } else {
                // SparkMax
                SparkMax m_flywheel = (SparkMax) getMotor();
                m_flywheel.setVoltage(volts.in(Volts));
            }
        }
    }

    /* Log Flywheel voltage, angular position and agular velocity */
    private void logFlywheel(SysIdRoutineLog log) {
        // TalonFX
        if (getMotor() instanceof TalonFX) {
            TalonFX m_flywheel = (TalonFX) getMotor();
            // Unit: Volts
            Logger.recordOutput("Flywheel/Voltage", m_flywheel.get() * RobotController.getBatteryVoltage());
            // Unit: Rotations
            Logger.recordOutput(
                    "Flywheel/AnglePosition", m_flywheel.getPosition().getValueAsDouble());
        } else {
            // SparkFlex
            if (getMotor() instanceof SparkFlex) {
                SparkFlex m_flywheel = (SparkFlex) getMotor();
                // Unit: Volts
                Logger.recordOutput("Flywheel/Voltage", m_flywheel.get() * RobotController.getBatteryVoltage());
                // Unit: Rotations
                Logger.recordOutput(
                        "Flywheel/AnglePosition", m_flywheel.getEncoder().getPosition());
            } else {
                // SparkMax
                SparkMax m_flywheel = (SparkMax) getMotor();
                // Unit: Volts
                Logger.recordOutput("Flywheel/Voltage", m_flywheel.get() * RobotController.getBatteryVoltage());
                // Unit: Rotations
                Logger.recordOutput(
                        "Flywheel/AnglePosition", m_flywheel.getEncoder().getPosition());
            }
        }
        // Unit: Rotations Per Second
        Logger.recordOutput("Flywheel/AngularVelocity", getEncoderRate() * 2.0 * Math.PI);
    }
}
