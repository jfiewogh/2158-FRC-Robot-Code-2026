package org.austincans.lib.subsystems.drive.swerve;

import static edu.wpi.first.units.Units.Meters;
import static edu.wpi.first.units.Units.MetersPerSecond;
import static edu.wpi.first.units.Units.RadiansPerSecond;
import static edu.wpi.first.units.Units.Volts;

import org.austincans.lib.io.ConfigRequests;
import org.austincans.lib.io.motors.MotorIO;
import org.austincans.lib.io.motors.inputs.MotorIOInputs;
import org.austincans.lib.io.motors.outputs.MotorIOOutputs;
import org.austincans.lib.io.motors.outputs.control.PositionControlInputs;
import org.austincans.lib.io.motors.outputs.control.VelocityControlInputs;
import org.austincans.lib.io.sensors.encoders.inputs.AbsoluteEncoderIOInputs;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.units.measure.Distance;
import edu.wpi.first.units.measure.Voltage;

// TOOD: What to do if I only want to run command on single module?
// It seems I would need to create a SwerveModule subsystem for the corresponding module.
// but maybe this can be avoided by using single Robot subsystem


public class SwerveModuleIO {
    private final MotorIO driveMotor;
    private final MotorIO steerMotor;

    public SwerveModuleIO(MotorIO driveMotor, MotorIO steerMotor) {
        this.driveMotor = driveMotor;
        this.steerMotor = steerMotor;
    }

    public class SwerveModuleInputs {
        public final MotorIOInputs driveMotorInputs;
        public final MotorIOInputs steerMotorInputs;
        public final AbsoluteEncoderIOInputs steerAbsoluteEncoderInputs;

        public SwerveModuleInputs(MotorIOInputs driveMotorInputs, MotorIOInputs steerMotorInputs, AbsoluteEncoderIOInputs steerAbsoluteEncoderInputs) {
            this.driveMotorInputs = driveMotorInputs;
            this.steerMotorInputs = steerMotorInputs;
            this.steerAbsoluteEncoderInputs = steerAbsoluteEncoderInputs;
        }
    }

    public class SwerveModuleOutputs {
        private final MotorIOOutputs driveMotorOutputs;
        private final MotorIOOutputs steerMotorOutputs;

        public SwerveModuleOutputs() {
            driveMotorOutputs = new MotorIOOutputs();
            steerMotorOutputs = new MotorIOOutputs();
        }

        public MotorIOOutputs getDriveMotorOutputs() {
            return driveMotorOutputs;
        }
        public MotorIOOutputs getSteerMotorOutputs() {
            return steerMotorOutputs;
        }

        private void setDriveOpenLoop(Voltage output) {
            driveMotorOutputs.setVoltage(output);
        }
        private void setSteerOpenLoop(Voltage output) {
            steerMotorOutputs.setVoltage(output);
        }

        private void setDriveVelocity(double velocityMetersPerSec) {
            driveMotorOutputs.setVelocity();
        }
        private void setSteerAngle(Rotation2d angle) {
            steerMotorOutputs.setPosition();
        }

        /**
         * Runs the module with the specified setpoint state.
         * Mutates the state to optimize it.
         */
        public void runSetpoint(SwerveModuleState state, Rotation2d currentSteerAngle, Distance wheelRadius) {
            // Optimize velocity setpoint
            state.optimize(currentSteerAngle);
            state.cosineScale(currentSteerAngle);

            // Apply setpoints
            AngularVelocity velocity = RadiansPerSecond.of(state.speedMetersPerSecond / wheelRadius.in(Meters));
            setDriveVelocity(velocity);
            setSteerAngle(state.angle.getMeasure());
        }

        /** Runs the module with the specified output while controlling to zero degrees. */
        public void runCharacterization(double output) {
            setDriveOpenLoop(Volts.of(output));
            setSteerAngle(Rotation2d.kZero);
        }

        /** Runs the module with the specified output toward Angular Motions */
        public void runAngularCharacterization(double output, Translation2d moduleLocationMeters) {
            setDriveOpenLoop(Volts.of(0));
            // TODO: maybe try to avoid creating new object (Rotation2d) during periodic
            setSteerAngle(
                new Rotation2d(moduleLocationMeters.getX(), moduleLocationMeters.getY())
                    .plus(Rotation2d.kCCW_Pi_2));
        }

        /** Disables all outputs to motors */
        public void stop() {
            setDriveOpenLoop(Volts.of(0));
            setSteerOpenLoop(Volts.of(0));
        }
    }

    public class SwerveModuleConfigRequests {
        public final ConfigRequests driveMotorConfigRequests;
        public final ConfigRequests steerMotorConfigRequests;

        public SwerveModuleConfigRequests(ConfigRequests driveMotorConfigRequests, ConfigRequests steerMotorConfigRequests) {
            this.driveMotorConfigRequests = driveMotorConfigRequests;
            this.steerMotorConfigRequests = steerMotorConfigRequests;
        }
    }

    public void updateInputs(SwerveModuleInputs inputs) {

    }

    public void applyOutputs(SwerveModuleOutputs outputs) {

    }

    public void applyConfigRequests(SwerveModuleConfigRequests configRequests) {

    }
}
