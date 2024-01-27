package frc.robot.subsystems;

import com.revrobotics.CANSparkBase.ControlType;
import com.revrobotics.CANSparkBase.SoftLimitDirection;

import org.littletonrobotics.junction.Logger;

import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.SparkPIDController;

import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.networktables.GenericEntry;
import edu.wpi.first.wpilibj.DoubleSolenoid;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.Model.ArmAngleLog;
import frc.robot.Model.ArmAngleLogAutoLogged;
import frc.robot.utilities.ArmAngle;
import frc.robot.utilities.MathUtils;
import frc.robot.utilities.SparkFactory;

public class ArmAngleSubsystem extends SubsystemBase {
    
    private CANSparkMax armMotor;
    
    private RelativeEncoder armEncoder;
    private SparkPIDController armPID;
    
    
    private final double tolerance = 0.1;
    private double setPoint;
    
    private final double goalConstant = 1.98 - Constants.LimlihConstants.limlihHeight;
    private GenericEntry setpointGE;
    private GenericEntry positionGE;
    private ArmAngleLogAutoLogged armAngleLogAutoLogged;
    
    public ArmAngleSubsystem() {
        
        armAngleLogAutoLogged = new ArmAngleLogAutoLogged();
        armMotor = SparkFactory.createCANSparkMax(Constants.CANIDConstants.armRotation1, true);
        armPID = armMotor.getPIDController();
        armEncoder = armMotor.getEncoder();
        
        armMotor.enableSoftLimit(SoftLimitDirection.kForward, true);
        armMotor.enableSoftLimit(SoftLimitDirection.kReverse, true);
        armMotor.setSoftLimit(SoftLimitDirection.kForward, ArmAngle.FULL.getValue());
        armMotor.setSoftLimit(SoftLimitDirection.kReverse, ArmAngle.ZERO.getValue());
        
        armEncoder.setPosition(0);
        armPID.setP(0.15);
        armPID.setI(0);
        armPID.setD(1.5);
        
        
        armEncoder.setPositionConversionFactor(1 / Constants.ArmAngleSubsystemConstants.armGearRatio);
        
        
        setpointGE = Shuffleboard.getTab("Arm Angle").add("setpoint", 0).getEntry();
        positionGE = Shuffleboard.getTab("Arm Angle").add("position", 0).getEntry();
        
    }
    
    public void setArmAngle(Pose3d pose) {
        double radians = Math.atan2(goalConstant, pose.getZ());
        radians = MathUtils.clamp(0, 1.22, radians);
        double ticksPerRad = ArmAngle.HORIZONTAL.getValue()/1.22;

        setPoint = radians * ticksPerRad;
    }
    
    public boolean atSetpoint() {
        return Math.abs(armEncoder.getPosition()-setPoint)<=tolerance;
        
    }

    private void updateInputs(ArmAngleLog armAngleLog) {
        armAngleLog.setpoint = setPoint;
        armAngleLog.position = armEncoder.getPosition();
        Logger.processInputs("Arm Angle", armAngleLogAutoLogged);

    }
    
    @Override
    public void periodic() {
        setpointGE.setDouble(setPoint);
        positionGE.setDouble(armEncoder.getPosition());
        armPID.setReference(setPoint, ControlType.kPosition);
        updateInputs(armAngleLogAutoLogged);
    }

    public void setArmAngle(ArmAngle armAngle) {

        setPoint = armAngle.getValue();


    }



}
