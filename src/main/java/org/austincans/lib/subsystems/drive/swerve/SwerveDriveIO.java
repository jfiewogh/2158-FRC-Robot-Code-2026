package org.austincans.lib.subsystems.drive.swerve;

import org.austincans.lib.subsystems.drive.swerve.SwerveModuleIO.SwerveModuleInputs;
import org.austincans.lib.subsystems.drive.swerve.SwerveModuleIO.SwerveModuleOutputs;

public class SwerveDriveIO {
    private final SwerveModuleIO frontLeft;
    private final SwerveModuleIO frontRight;
    private final SwerveModuleIO backLeft;
    private final SwerveModuleIO backRight;

    public SwerveDriveIO(SwerveModuleIO frontLeft, SwerveModuleIO frontRight, SwerveModuleIO backLeft, SwerveModuleIO backRight) {
        this.frontLeft = frontLeft;
        this.frontRight = frontRight;
        this.backLeft = backLeft;
        this.backRight = backRight;
    }

    public class SwerveDriveInputs {
        public final SwerveModuleInputs frontLeftModuleInputs;
        public final SwerveModuleInputs frontRightModuleInputs;
        public final SwerveModuleInputs backLeftModuleInputs;
        public final SwerveModuleInputs backRightModuleInputs;

        public SwerveDriveInputs(SwerveModuleInputs frontLeft, SwerveModuleInputs frontRight, SwerveModuleInputs backLeft, SwerveModuleInputs backRight) {
            frontLeftModuleInputs = frontLeft;
            frontRightModuleInputs = frontRight;
            backLeftModuleInputs = backLeft;
            backRightModuleInputs = backRight;
        }
    }

    public class SwerveDriveOutputs {
        public final SwerveModuleOutputs frontLeftModuleOutputs;
        public final SwerveModuleOutputs frontRightModuleOutputs;
        public final SwerveModuleOutputs backLeftModuleOutputs;
        public final SwerveModuleOutputs backRightModuleOutputs;
        
        public SwerveDriveOutputs() {
            frontLeftModuleOutputs = new SwerveModuleOutputs();
            frontRightModuleOutputs = new SwerveModuleOutputs();
            backLeftModuleOutputs = new SwerveModuleOutputs();
            backRightModuleOutputs = new SwerveModuleOutputs();
        }
    }

    public void updateInputs(SwerveDriveInputs inputs) {
        frontLeft.updateInputs(inputs.frontLeftModuleInputs);
        frontRight.updateInputs(inputs.frontRightModuleInputs);
        backLeft.updateInputs(inputs.backLeftModuleInputs);
        backRight.updateInputs(inputs.backRightModuleInputs);
    }

    public void applyOutputs(SwerveDriveOutputs outputs) {
        frontLeft.applyOutputs(outputs.frontLeftModuleOutputs);
        frontRight.applyOutputs(outputs.frontRightModuleOutputs);
        backLeft.applyOutputs(outputs.backLeftModuleOutputs);
        backRight.applyOutputs(outputs.backRightModuleOutputs);
    }

    public void applyConfigRequests() {
        
    }
}
