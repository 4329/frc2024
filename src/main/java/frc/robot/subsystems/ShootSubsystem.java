package frc.robot.subsystems;

import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.SparkPIDController;

import org.littletonrobotics.junction.Logger;
import org.littletonrobotics.junction.inputs.LoggableInputs;

import com.revrobotics.CANSparkBase.IdleMode;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.networktables.GenericEntry;
import edu.wpi.first.math.controller.BangBangController;
import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.units.BaseUnits;
import edu.wpi.first.units.Measure;
import edu.wpi.first.units.Units;
import edu.wpi.first.units.Voltage;
import edu.wpi.first.wpilibj.RobotController;
import edu.wpi.first.wpilibj.shuffleboard.BuiltInWidgets;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.sysid.SysIdRoutineLog;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.Model.ShootLog;
import frc.robot.Model.ShootLogAutoLogged;
import frc.robot.subsystems.LoggingSubsystem.LoggedSubsystem;
import frc.robot.utilities.HoorayConfig;
import frc.robot.utilities.LinearInterpolationTable;
import frc.robot.utilities.SparkFactory;

import java.awt.Point;
import java.awt.geom.Point2D;

public class ShootSubsystem extends SubsystemBase implements LoggedSubsystem {

    public final CANSparkMax rightMotor;
    public final CANSparkMax leftMotor;
    public final RelativeEncoder rightEncoder;
    public final RelativeEncoder leftEncoder;
    public final SparkPIDController m_aimBot;
    private GenericEntry rpmSetpointGE;
    private GenericEntry rpmActualGE;
    private GenericEntry rpmActual2GE;

    private double setpoint = 0;
    private double tolerance = 50; //arbitrary

    private ShootLogAutoLogged shootLogAutoLogged;

    // private double kP = 0.0000432;
    // private double kI = 0.000001;
    // private double kD = 0.00017;
    // private double kFF = 0.00018;
    // private double kIZ = 67;
    private double kP = 0.0000432;
    private double kI = 0.000001;
    private double kD = 0.00017;
    private double kFF = 0.00018;
    private double kIZ = 67;


   
    LinearInterpolationTable shootTable = new LinearInterpolationTable(
        new Point(0, 0)
    );
    

    // 240 inches is the theroetical max shot for the shooter
    public ShootSubsystem() {
        rightMotor = SparkFactory.createCANSparkMax(Constants.CANIDConstants.shoot1);
        leftMotor = SparkFactory.createCANSparkMax(Constants.CANIDConstants.shoot2);
        m_aimBot = rightMotor.getPIDController();
        rightEncoder = rightMotor.getEncoder();
        leftEncoder = leftMotor.getEncoder();

        leftMotor.follow(rightMotor, true);

        rightMotor.enableVoltageCompensation(11.8);
        leftMotor.enableVoltageCompensation(12.8);

        rightMotor.setIdleMode(IdleMode.kCoast);
        leftMotor.setIdleMode(IdleMode.kCoast);

        rightMotor.burnFlash();
        leftMotor.burnFlash();
        

        m_aimBot.setP(kP);
        m_aimBot.setI(kI);
        m_aimBot.setD(kD);
        m_aimBot.setFF(kFF);
        m_aimBot.setIZone(kIZ);

        
            HoorayConfig.gimmeConfig().getShooterkV();
        

        shootLogAutoLogged = new ShootLogAutoLogged();

        
        rpmActualGE =  Shuffleboard.getTab("shoot").add("rpm actual 1", 0).getEntry();
        rpmSetpointGE = Shuffleboard.getTab("shoot").add("rpm setpoint", 0).getEntry();
        
        rpmActual2GE = Shuffleboard.getTab("shoot").add("rpm actual 2", 0).getEntry();
    }

    public void changeSetpoint(double set) {
        this.setpoint = set;

    }

    public boolean atSetpoint() {

        if (Math.abs(setpoint - getVelocity()) <= tolerance) {
          System.out.println("atsetpoint ----");

            return true;
        }
        return false;


    }

    public boolean aboveSetpoint() {

        if (rightEncoder.getVelocity() >= setpoint) {
            return true;
        }
        return false;

    }

    public void stop(){
        System.out.println("shootSTOP");

       setpoint = 0;
       rightMotor.stopMotor();
    }
    @Override
    public LoggableInputs log() {
        shootLogAutoLogged.setpoint = setpoint;
        shootLogAutoLogged.PIDOutput = rightMotor.get();
        return shootLogAutoLogged;
    }

    @Override
    public void periodic() {
        

        // setpoint = sadf.getDouble(0);


        rpmActualGE.setDouble(rightEncoder.getVelocity());
        rpmActual2GE.setDouble(leftEncoder.getVelocity());
        rpmSetpointGE.setDouble(setpoint);

        if (setpoint == 0) {

            rightMotor.stopMotor();
            leftMotor.stopMotor();
        } else {
            m_aimBot.setReference(setpoint, CANSparkMax.ControlType.kVelocity);
        }
    }
    public void setRPM(double rpm) {
        setpoint = rpm;
    }

    public void setVoltage(Measure<Voltage> voltage) {
        rightMotor.setVoltage(voltage.in(BaseUnits.Voltage));
    }

    public void getData(SysIdRoutineLog sysIdRoutineLog) {
        Logger.recordOutput("sdifa", RobotController.getBatteryVoltage());
        Logger.recordOutput("lsdoflsoaolodsflsdfldlsoflso", rightMotor.getAppliedOutput());

        rightMotor.getVoltageCompensationNominalVoltage();
        sysIdRoutineLog.motor("Shoot")
                .voltage(BaseUnits.Voltage.of(rightMotor.getAppliedOutput() * RobotController.getBatteryVoltage()))
                .angularPosition(Units.Rotations.of(rightEncoder.getPosition()))
                .angularVelocity(Units.RotationsPerSecond.of(rightEncoder.getVelocity() / 60));
    }

    public double getVelocity() {
        return rightEncoder.getVelocity();
    }
}

