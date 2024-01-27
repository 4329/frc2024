package frc.robot.subsystems;

import com.revrobotics.CANSparkBase.ControlType;
import com.revrobotics.CANSparkBase.SoftLimitDirection;
import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.SparkPIDController;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.utilities.ArmAngle;
import frc.robot.utilities.ElevatorSetpoints;
import frc.robot.utilities.SparkFactory;

public class ElevatorSubsystem extends SubsystemBase {

    private CANSparkMax elevatorMotor1;
    private CANSparkMax elevatorMotor2;
    private RelativeEncoder elevatorEncoder;
    private SparkPIDController elevatorPID;

    private double setPoint;
    private final double tolerance = 0.1;


    public ElevatorSubsystem() {

        elevatorMotor1 = SparkFactory.createCANSparkMax(Constants.CANIDConstants.elevatorMotor1, false);
        elevatorMotor2 = SparkFactory.createCANSparkMax(Constants.CANIDConstants.elevatorMotor2, false);

        elevatorPID = elevatorMotor1.getPIDController();
        elevatorEncoder = elevatorMotor1.getEncoder();
        elevatorMotor1.enableSoftLimit(SoftLimitDirection.kForward, true);
        elevatorMotor1.enableSoftLimit(SoftLimitDirection.kReverse, true);
        elevatorMotor1.setSoftLimit(SoftLimitDirection.kForward, ElevatorSetpoints.ONEHUNDRED.getValue());
        elevatorMotor1.setSoftLimit(SoftLimitDirection.kReverse, ElevatorSetpoints.ZERO.getValue());

        elevatorMotor2.follow(elevatorMotor1, true);

        elevatorEncoder.setPosition(0);

        elevatorPID.setP(0.1);
        elevatorPID.setI(0);
        elevatorPID.setD(0.1);
        elevatorPID.setFF(0);

        
        // elevatorEncoder.setPositionConversionFactor(1 / Constants.ArmAngleSubsystemConstants.armGearRatio);
        elevatorMotor1.burnFlash();
        elevatorMotor2.burnFlash();
    }

    public void setElevatorPose(ElevatorSetpoints elevatorSetpoints) {

        setPoint = elevatorSetpoints.getValue();

    }

    public boolean atSetpoint() {

        return Math.abs(elevatorEncoder.getPosition() - setPoint) <= tolerance;

    }

    @Override
    public void periodic() {

    elevatorPID.setReference(setPoint, ControlType.kPosition);


    }

}
