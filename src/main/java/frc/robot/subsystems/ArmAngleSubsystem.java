package frc.robot.subsystems;

import com.revrobotics.CANSparkBase.ControlType;
import com.revrobotics.CANSparkBase.SoftLimitDirection;
import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.SparkPIDController;

import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.networktables.GenericEntry;
import edu.wpi.first.wpilibj.DoubleSolenoid;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.utilities.SparkFactory;

public class ArmAngleSubsystem extends SubsystemBase {

    private CANSparkMax armMotor;

    private RelativeEncoder armEncoder;
    private SparkPIDController armPID;

    private boolean brake;

    private float maxValue;
    private float minValue;

    private double radians;
    private final double tolerance;

    private GenericEntry setpointGE;
    private GenericEntry positionGE;

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

        armEncoder.setPositionConversionFactor(1 / Constants.ArmAngleSubsystemConstants.armGearRatio);
        tolerance = 0.1;

        setpointGE = Shuffleboard.getTab("Arm Angle").add("setpoint", 0).getEntry();
        positionGE = Shuffleboard.getTab("Arm Angle").add("position", 0).getEntry();
        
    }

    public void setArmAngle(Pose3d pose) {
        radians = Math.atan2(pose.getY(), pose.getZ()*-1);
        
    }
    
    public boolean atSetpoint() {
        return Math.abs(armEncoder.getPosition()-radians)<=tolerance;
        
    }
    
    @Override
    public void periodic() {
        setpointGE.setDouble(radians);
        positionGE.setDouble(armEncoder.getPosition());
        armPID.setReference(radians, ControlType.kPosition);
    }

    
   
    

}
