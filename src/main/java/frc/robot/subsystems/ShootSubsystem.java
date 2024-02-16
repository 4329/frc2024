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
import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.networktables.GenericEntry;
import edu.wpi.first.units.Angle;
import edu.wpi.first.units.BaseUnits;
import edu.wpi.first.units.Measure;
import edu.wpi.first.units.MutableMeasure;
import edu.wpi.first.units.Units;
import edu.wpi.first.units.Voltage;
import edu.wpi.first.wpilibj.RobotController;
import edu.wpi.first.wpilibj.shuffleboard.BuiltInWidgets;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj.sysid.SysIdRoutineLog;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine;
import frc.robot.Constants;
import frc.robot.RobotContainer;
import frc.robot.Model.ShootLog;
import frc.robot.Model.ShootLogAutoLogged;
import frc.robot.utilities.Config;
import frc.robot.utilities.SometimesTextSendable;
import frc.robot.utilities.SparkFactory;

public class ShootSubsystem extends SubsystemBase {

    public final CANSparkMax m_rightShoot;
    public final CANSparkMax m_leftShoot;
    public final RelativeEncoder m_shootEncoderRight;
    public final RelativeEncoder m_shootEncoderLeft;
    // public final SparkPIDController m_aimBot;
    // public final PIDController m_aimBot;
    private static final int rightID = 13;
    private double P = 0.5;
    private double I = 0;
    private double D = 0.0;
    private double maxRPM = 5700;
    private double setpoint = 0;
    private double PIDOutput = 0;

    private double kP = 0.0004;
    private double kI = 0.00001;
    private double kD = 3;
    private double kIz = 67;
    private double kFF = 0.00015;
    private double kMaxOutput = 1;
    private double kMinOutput = -1;
    private ShootLogAutoLogged shootLogAutoLogged;
    
    private SometimesTextSendable someTimesPoint;
    private GenericEntry shoot;
    
    private SimpleMotorFeedforward aimFeed;

    // 240 inches is the theroetical max shot for the shooter
    public ShootSubsystem() {
        m_rightShoot = SparkFactory.createCANSparkMax(rightID, false);
        m_leftShoot = SparkFactory.createCANSparkMax(12, false);
        // m_aimBot = m_rightShoot.getPIDController();
        // m_aimBot = new PIDController(P, I, D);
        m_shootEncoderRight = m_rightShoot.getEncoder();
        m_shootEncoderLeft = m_leftShoot.getEncoder();
        // m_aimBot.setP(kP);
        // m_aimBot.setI(kI);
        // m_aimBot.setD(kD);
        // m_aimBot.setIZone(kIz);
        // m_aimBot.setFF(kFF);
        // m_aimBot.setOutputRange(kMinOutput, kMaxOutput);
        m_leftShoot.follow(m_rightShoot, true);
        m_rightShoot.enableVoltageCompensation(Constants.voltageCompensation);
        m_leftShoot.enableVoltageCompensation(Constants.voltageCompensation);
        shootLogAutoLogged = new ShootLogAutoLogged();
        
        // Shuffleboard.getTab("shooterTuningExceptWeDontNeedThatAnymoreBecauseBangBangIsAwesome").add("PIDControlerWhichIsn'tNeededAnymore", m_aimBot);

        m_rightShoot.setIdleMode(IdleMode.kCoast);
        m_leftShoot.setIdleMode(IdleMode.kCoast);
        m_rightShoot.burnFlash();
        m_leftShoot.burnFlash();

        aimFeed = new SimpleMotorFeedforward(0.52273, 0.12816);

        someTimesPoint = new SometimesTextSendable();
        Shuffleboard.getTab("dumbstpid").add("setpoint", someTimesPoint);
        shoot = Shuffleboard.getTab("dumbspid").add("out", 0).getEntry();
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
        updateInputs(shootLogAutoLogged);

        setpoint = someTimesPoint.sometimesGet(setpoint);
        shoot.setDouble(m_shootEncoderRight.getVelocity());

        if (setpoint == 0) {

            m_rightShoot.stopMotor();
            m_leftShoot.stopMotor();

        } else {
            m_rightShoot.set(aimFeed.calculate(setpoint));
        }
    }

    public void setRPM(double rpm) {
        setpoint = rpm;
    }

    public void setVoltage(Measure<Voltage> voltage) {
        m_rightShoot.setVoltage(voltage.in(BaseUnits.Voltage));
    }

    public void getData(SysIdRoutineLog sysIdRoutineLog) {
        Logger.recordOutput("sdifa", RobotController.getBatteryVoltage());
        Logger.recordOutput("lsdoflsoaolodsflsdfldlsoflso", m_rightShoot.getAppliedOutput());

        m_rightShoot.getVoltageCompensationNominalVoltage();
        sysIdRoutineLog.motor("Shoot")
                .voltage(BaseUnits.Voltage.of(m_rightShoot.getAppliedOutput() * RobotController.getBatteryVoltage()))
                .angularPosition(Units.Rotations.of(m_shootEncoderRight.getPosition()))
                .angularVelocity(Units.RotationsPerSecond.of(m_shootEncoderRight.getVelocity() / 60));
    }
}
