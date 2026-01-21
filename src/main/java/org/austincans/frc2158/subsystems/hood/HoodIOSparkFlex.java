package org.austincans.frc2158.subsystems.hood;
import org.austincans.frc2158.math.Angle;
import org.austincans.frc2158.subsystems.selfcheck.SelfCheckingSparkFlex;

import com.revrobotics.RelativeEncoder;
import com.revrobotics.spark.SparkClosedLoopController;
import com.revrobotics.spark.SparkFlex;
import com.revrobotics.spark.SparkBase.ControlType;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.util.Units;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.config.SparkFlexConfig;

public class HoodIOSparkFlex implements HoodIO{
    private SparkFlex hoodMotor = new SparkFlex(HoodConstants.flexMotorID, MotorType.kBrushless);
    private SelfCheckingSparkFlex hoodMotorCheck = new SelfCheckingSparkFlex("Hood Motor", hoodMotor);
    private RelativeEncoder motorEncoder = hoodMotor.getEncoder();
    SparkClosedLoopController controller = hoodMotor.getClosedLoopController();

    SparkFlexConfig config = new SparkFlexConfig();
    
    public HoodIOSparkFlex(){
        config.closedLoop
        .p(HoodConstants.kP)
        .i(HoodConstants.kI)
        .d(HoodConstants.kD);
        // .outputRange(kMinOutput, kMaxOutput);
    }

    @Override
    public Angle getAngle() {
        return Angle.fromRadians(Units.rotationsToRadians(motorEncoder.getPosition()));
    }
    
    @Override
    public Rotation2d getAngleRotation2d() {
        return Rotation2d.fromRadians(Units.rotationsToRadians(motorEncoder.getPosition()));
    }

    public void runHoodMotor(double speed){
        hoodMotor.set(speed);
    }

    public void setAngle(Angle setPointDegrees){
        controller.setSetpoint(Units.degreesToRotations(setPointDegrees.toDegrees()), ControlType.kPosition);
    }

    public void updateInputs(hoodIOInputs inputs){
        inputs.hoodAngleDegrees = getAngle();
        inputs.temperature = hoodMotor.getMotorTemperature();
        inputs.motorPosition = motorEncoder.getPosition();
    }

    public void setAngleRotation2d(Rotation2d angle){
        controller.setSetpoint(Units.degreesToRotations(angle.getDegrees()), ControlType.kPosition);
    }

    @Override
    public boolean atAngle(Angle setPoint){
        if (getAngle() == setPoint){
            return true;
        }
        else{
            return false;
        }
    }
}
