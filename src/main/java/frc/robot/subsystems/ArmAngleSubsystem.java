package frc.robot.subsystems;

import com.revrobotics.CANSparkBase.SoftLimitDirection;
import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.SparkPIDController;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.utilities.SparkFactory;

public class ArmAngleSubsystem extends SubsystemBase {

    private CANSparkMax armMotor;

    private RelativeEncoder armEncoder;
    private SparkPIDController armPID;
    
    private boolean braking;

    private float maxValue;
    private float minValue;

    public ArmAngleSubsystem() {

        maxValue = 60f;
        minValue = 15f;

        armMotor = SparkFactory.createCANSparkMax(Constants.CANIDConstants.armRotation1, true);
        armPID = armMotor.getPIDController();
        armEncoder = armMotor.getEncoder();

        armMotor.enableSoftLimit(SoftLimitDirection.kForward, true);
        armMotor.enableSoftLimit(SoftLimitDirection.kReverse, true);
        armMotor.setSoftLimit(SoftLimitDirection.kForward, maxValue);
        armMotor.setSoftLimit(SoftLimitDirection.kReverse, minValue);

        armEncoder.setPosition(0);
        armPID.setP(0.5);
        armPID.setI(0);
        armPID.setD(0.5);

        
    }

    private double armAngleRadians(double distance) {
        double angle = Math.atan2(Constants.LimlihConstants.limlihHeight, distance);
        return angle;
    }
}
