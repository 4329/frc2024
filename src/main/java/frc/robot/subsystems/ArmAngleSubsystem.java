package frc.robot.subsystems;

import com.revrobotics.CANSparkBase.ControlType;
import com.revrobotics.CANSparkBase.IdleMode;
import com.revrobotics.CANSparkBase.SoftLimitDirection;
import com.revrobotics.SparkLimitSwitch.Type;

import org.littletonrobotics.junction.Logger;
import org.littletonrobotics.junction.inputs.LoggableInputs;

import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.SparkPIDController;

import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.networktables.GenericEntry;
import edu.wpi.first.wpilibj.DoubleSolenoid;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.Model.ArmAngleLog;
import frc.robot.Model.ArmAngleLogAutoLogged;
import frc.robot.subsystems.LoggingSubsystem.LoggedSubsystem;
import frc.robot.utilities.ArmAngle;
import frc.robot.utilities.BuiltOutWidgets;
import frc.robot.utilities.MathUtils;
import frc.robot.utilities.SometimesTextSendable;
import frc.robot.utilities.SparkFactory;

public class ArmAngleSubsystem extends SubsystemBase implements LoggedSubsystem {

    private CANSparkMax armMotor;

    private RelativeEncoder armEncoder;
    private SparkPIDController armPID;

    private boolean brake;

    private final double tolerance = 0.1;
    private double setpoint = 0;

    double ticksPerRad = ArmAngle.HORIZONTAL.getValue() / ((59.5 * Math.PI) / (180.0));

    private final double speakerHeight = 2.15;
    private double speakerMod = 0;
    private final double goalConstant = speakerHeight - Constants.LimlihConstants.limlihHeight;
    private GenericEntry setpointGE;
    private GenericEntry positionGE;
    private GenericEntry radiansRotatedGE;
    private GenericEntry radians2RotatedGE;
    private GenericEntry speakerModGE;
    private ArmAngleLogAutoLogged armAngleLogAutoLogged;


    
    public ArmAngleSubsystem() {

        armAngleLogAutoLogged = new ArmAngleLogAutoLogged();
        armMotor = SparkFactory.createCANSparkMax(Constants.CANIDConstants.armRotation1, false);
        armPID = armMotor.getPIDController();
        armEncoder = armMotor.getEncoder();

        armMotor.enableSoftLimit(SoftLimitDirection.kForward, true);
        armMotor.enableSoftLimit(SoftLimitDirection.kReverse, true);
        armMotor.setIdleMode(IdleMode.kBrake);
        armMotor.setSoftLimit(SoftLimitDirection.kForward, (float) ArmAngle.FULL.getValue());
        armMotor.setSoftLimit(SoftLimitDirection.kReverse, (float) ArmAngle.ZERO.getValue());
        armMotor.enableVoltageCompensation(Constants.voltageCompensation);

        armEncoder.setPosition(0);
        armPID.setP(0.5);
        armPID.setI(0);
        armPID.setD(1.75);
        armPID.setFF(0);
        armPID.setOutputRange(-0.15, 0.15); 

        armEncoder.setPositionConversionFactor(1 / Constants.ArmAngleSubsystemConstants.armGearRatio);

        setpointGE = Shuffleboard.getTab("Arm Angle").add("arm setpoint", 0).getEntry();
        positionGE = Shuffleboard.getTab("Arm Angle").add("arm position", 0).getEntry();
        radiansRotatedGE = Shuffleboard.getTab("Arm Angle").add("RadiansRotated", 0).getEntry();
        radians2RotatedGE = Shuffleboard.getTab("Arm Angle").add("Radians2Rotated", 0).getEntry();
        speakerModGE = Shuffleboard.getTab("Arm Angle").add("SpeakerMod", 0).getEntry();

        armMotor.burnFlash();
    }

    public void incrementSetpoint(double increment) {
        setpoint += increment;
    }

    public boolean endSensor() {

        return armMotor.getReverseLimitSwitch(Type.kNormallyOpen).isPressed();
    }

    public void resetZero() {

        setpoint = 0;
        armEncoder.setPosition(0);
    }

    public void setArmAngle(Pose3d pose) {

        double radians1 = Math.atan2(goalConstant, pose.getZ());
        speakerMod = speakerHeight - (radians1 * 0.2);
        radians1 = MathUtils.clamp(0, 1.03, radians1); //was 1.22
        double radians2 = Math.atan2(speakerMod, pose.getZ());
        speakerModGE.setDouble(speakerMod);
        radiansRotatedGE.setDouble(radians1);
        radians2RotatedGE.setDouble(radians2);



        setpoint = ArmAngle.HORIZONTAL.getValue() - (radians2 * ticksPerRad);
    }

    public boolean atSetpoint() {
        return Math.abs(armEncoder.getPosition() - setpoint) <= tolerance;

    }
    @Override
    public LoggableInputs log() {
        armAngleLogAutoLogged.setpoint = setpoint;
        armAngleLogAutoLogged.position = armEncoder.getPosition();
        armAngleLogAutoLogged.radians = armEncoder.getPosition() / 4.1284; //subject to change
        return armAngleLogAutoLogged;
    }

    public void armPositonUp() {
        if (setpoint < ArmAngle.ARMAMP.getValue() - 0.05) {
            setpoint = Math.min(setpoint + 0.05, ArmAngle.ARMAMP.getValue());
        }
        else {
            setpoint = ArmAngle.FULL.getValue();
        }
    }

    public void armPositonDown() {
        if (setpoint > ArmAngle.ZERO.getValue() + 0.05) {
            setpoint = Math.max(setpoint - 0.05, ArmAngle.ZERO.getValue());
        }
        else {
            setpoint = ArmAngle.ZERO.getValue();
        }
    }

    @Override
    public void periodic() {
        setpointGE.setDouble(setpoint);
        positionGE.setDouble(armEncoder.getPosition());
        armPID.setReference(setpoint, ControlType.kPosition);
    }

    public void setArmAngle(ArmAngle armAngle) {

        setpoint = armAngle.getValue();
    }



    public double getAngleRadians() {
        return setpoint / ticksPerRad;
    }

}
