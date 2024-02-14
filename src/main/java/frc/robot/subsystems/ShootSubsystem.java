package frc.robot.subsystems;

import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.SparkPIDController;

import java.io.File;
import java.io.PrintWriter;

import org.littletonrobotics.junction.Logger;
import org.littletonrobotics.junction.wpilog.WPILOGWriter;

import com.revrobotics.CANSparkBase.IdleMode;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.networktables.GenericEntry;
import edu.wpi.first.units.Measure;
import edu.wpi.first.units.Voltage;
import edu.wpi.first.wpilibj.shuffleboard.BuiltInWidgets;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj.sysid.SysIdRoutineLog;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine;
import frc.robot.Constants;
import frc.robot.Model.ShootLog;
import frc.robot.Model.ShootLogAutoLogged;
import frc.robot.utilities.Config;
import frc.robot.utilities.SparkFactory;


public class ShootSubsystem extends SubsystemBase {

    public final CANSparkMax m_rightShoot;
    public final CANSparkMax m_leftShoot;
    public final RelativeEncoder m_shootEncoderRight;
    public final RelativeEncoder m_shootEncoderLeft;
    public final SparkPIDController m_aimBot;
    //public final PIDController m_aimBot;
    private static final int rightID = 13;
    private double P = 0.5;
    private double I = 0;
    private double D = 0.0;
    private double maxRPM = 5700;
    private double setpoint = 0;
    private double PIDOutput = 0;

    GenericEntry p;
    GenericEntry i;
    GenericEntry d;
    GenericEntry max;
    GenericEntry min;
    GenericEntry set;
    GenericEntry outPut;
    GenericEntry outPut1;

    GenericEntry iz;
    GenericEntry ff;
    GenericEntry rpmEntry;
    private double kP = 0.0004;
    private double kI = 0.00001;
    private double kD = 3;
    private double kIz = 67;
    private double kFF = 0.00015;
    private double kMaxOutput = 1;
    private double kMinOutput = -1;
    private ShootLogAutoLogged shootLogAutoLogged;
    private PrintWriter printWriter;

    // 240 inches is the theroetical max shot for the shooter
    public ShootSubsystem() {
        m_rightShoot = SparkFactory.createCANSparkMax(rightID, false);
        m_leftShoot = SparkFactory.createCANSparkMax(12, false);
        m_aimBot = m_rightShoot.getPIDController();
        //m_aimBot = new PIDController(P, I, D);
        m_shootEncoderRight = m_rightShoot.getEncoder();
        m_shootEncoderLeft = m_leftShoot.getEncoder();
        m_aimBot.setP(kP);
        m_aimBot.setI(kI);
        m_aimBot.setD(kD);
        m_aimBot.setIZone(kIz);
        m_aimBot.setFF(kFF);
        m_aimBot.setOutputRange(kMinOutput, kMaxOutput);
        m_leftShoot.follow(m_rightShoot, true);
        m_rightShoot.enableVoltageCompensation(Constants.voltageCompensation);
        m_leftShoot.enableVoltageCompensation(Constants.voltageCompensation);
        shootLogAutoLogged = new ShootLogAutoLogged();

        p = Shuffleboard.getTab("shoot").add("P Gain", 0).getEntry();
        i = Shuffleboard.getTab("shoot").add("I Gain", 0).getEntry();
        d = Shuffleboard.getTab("shoot").add("D Gain", 0).getEntry();

        set = Shuffleboard.getTab("shoot").add("Setpoint", 0).getEntry();
        outPut = Shuffleboard.getTab("shoot").add("outPut", 0).withWidget(BuiltInWidgets.kGraph).getEntry();
        outPut1 = Shuffleboard.getTab("shoot").add("outPutValue", 0).getEntry();

        m_rightShoot.setIdleMode(IdleMode.kCoast);
        m_leftShoot.setIdleMode(IdleMode.kCoast);
        m_rightShoot.burnFlash();
        m_leftShoot.burnFlash();
    }

    public void changeSetpoint(double set) {
        this.setpoint = set; 
            
    }

    public void stop(){
       m_rightShoot.stopMotor();
    }

    private void updateInputs(ShootLog shootLog) {
        shootLog.setpoint = setpoint;
        shootLog.PIDOutput = m_rightShoot.get();
        Logger.processInputs("Shooter", shootLogAutoLogged);

    }

    @Override
    public void periodic() {

        outPut.setDouble(m_shootEncoderRight.getVelocity());
        outPut1.setDouble(m_shootEncoderRight.getVelocity());
        updateInputs(shootLogAutoLogged);
        
        if (setpoint == 0) {

            m_rightShoot.stopMotor();
            m_leftShoot.stopMotor();
        }  else {
            
            // m_aimBot.setReference(setpoint, CANSparkMax.ControlType.kVelocity);

        }
    }

    public void setRPM(double rpm) {
        setpoint = rpm;
    }

    public void setVoltage(Measure<Voltage> voltage) {
        m_rightShoot.setVoltage(voltage.baseUnitMagnitude());
    }

}
