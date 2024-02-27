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
    public final SparkPIDController m_aimBot;

    private double setpoint = 0;

    private ShootLogAutoLogged shootLogAutoLogged;

    private double kP = 0.0004;
    private double kI = 0.00001;
    private double kD = 3;
    private double kFF = 0.00015;

   
    LinearInterpolationTable shootTable = new LinearInterpolationTable(
        new Point(0, 0)
    );
    

    // 240 inches is the theroetical max shot for the shooter
    public ShootSubsystem() {
        rightMotor = SparkFactory.createCANSparkMax(Constants.CANIDConstants.shoot1);
        leftMotor = SparkFactory.createCANSparkMax(Constants.CANIDConstants.shoot2);
        m_aimBot = rightMotor.getPIDController();
        rightEncoder = rightMotor.getEncoder();

        leftMotor.follow(rightMotor, true);

        rightMotor.enableVoltageCompensation(Constants.voltageCompensation);
        leftMotor.enableVoltageCompensation(Constants.voltageCompensation);

        rightMotor.setIdleMode(IdleMode.kCoast);
        leftMotor.setIdleMode(IdleMode.kCoast);

        rightMotor.burnFlash();
        leftMotor.burnFlash();
        

        m_aimBot.setP(kP);
        m_aimBot.setI(kI);
        m_aimBot.setD(kD);
        m_aimBot.setFF(kFF);

        
            HoorayConfig.gimmeConfig().getShooterkV();
        

        shootLogAutoLogged = new ShootLogAutoLogged();
        
    }

    public void changeSetpoint(double set) {
        this.setpoint = set;

    }

    public boolean atSetpoint() {

        if (setpoint == setpoint) {
            return true;
        }
        return false;


    }

    public void stop(){
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

