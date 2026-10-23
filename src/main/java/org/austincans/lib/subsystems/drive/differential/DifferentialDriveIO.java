package org.austincans.lib.subsystems.drive.differential;

import org.austincans.lib.io.motors.inputs.MotorIOInputs;
import org.austincans.lib.io.motors.outputs.MotorIOOutputs;
import org.littletonrobotics.junction.AutoLog;

public class DifferentialDriveIO {
    public class DifferentialDriveIOInputs {
        public final MotorIOInputs leftLeaderMotorInputs;
        public final MotorIOInputs leftFollowerMotorInputs;
        public final MotorIOInputs rightLeaderMotorInputs;
        public final MotorIOInputs rightFollowerMotorInputs;

        public DifferentialDriveIOInputs(MotorIOInputs leftLeaderMotorInputs, MotorIOInputs) {

        }
    }

    public class DifferentialDriveIOOutputs {
        public MotorIORequests leftLeaderMotorRequests = new MotorIORequests();
        public MotorIORequests rightLeaderMotorRequests = new MotorIORequests();

        public void setVelocities(double leftRadPerSec, double rightRadPerSec, double leftFFVolts, double rightFFVolts) {
            leftLeaderMotorRequests.
        }
        public void setVoltages(double leftVolts, double rightVolts) {
            leftLeaderMotorRequests.setVoltage(leftVolts);
            rightLeaderMotorRequests.setVoltage(rightVolts);
        }
    }

    public void updateInputs(DifferentialDriveIOInputs inputs) {

    }

    public void applyRequests(DifferentialDriveIORequests requests);
}
