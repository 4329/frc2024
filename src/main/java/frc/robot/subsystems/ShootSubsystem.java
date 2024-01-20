package frc.robot.subsystems;

import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.SparkPIDController;
import com.revrobotics.CANSparkBase.IdleMode;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.networktables.GenericEntry;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.utilities.SparkFactory;

public class ShootSubsystem extends SubsystemBase {

    public final CANSparkMax m_rightShoot;
    public final CANSparkMax m_leftShoot;
    public final RelativeEncoder m_shootEncoderRight;
    public final RelativeEncoder m_shootEncoderLeft;
    //public final SparkPIDController m_aimBot;
    public final PIDController m_aimBot;
    private static final int rightID = 13;
    private double P = 0.5;
    private double I = 0;
    private double D = 0.0;
    // private double kP = 6e-5;
    // private double kI = 0;
    // private double kD = 0;
    // private double kIz = 0;
    // private double kFF = 0.000015;
    // private double kMaxOutput = 1;
    // private double kMinOutput = -1;
    private double maxRPM = 5700;
    private double setpoint = 0.0;
    GenericEntry p;
    GenericEntry i;
    GenericEntry d;
    // GenericEntry iz;
    // GenericEntry ff;
    GenericEntry max;
    GenericEntry min;
    GenericEntry set;

    // 240 inches is the theroetical max shot for the shooter
    public ShootSubsystem() {
        m_rightShoot = SparkFactory.createCANSparkMax(rightID, false);
        m_leftShoot = SparkFactory.createCANSparkMax(12, true);
        //m_aimBot = m_rightShoot.getPIDController();
        m_aimBot = new PIDController(P, I, D);
        m_shootEncoderRight = m_rightShoot.getEncoder();
        m_shootEncoderLeft = m_leftShoot.getEncoder();
        // m_aimBot.setP(kP);
        // m_aimBot.setI(kI);
        // m_aimBot.setD(kD);
        // m_aimBot.setIZone(kIz);
        // m_aimBot.setFF(kFF);
        //m_aimBot.setOutputRange(kMinOutput, kMaxOutput);

        // display PID coefficients on SmartDashboard

        p = Shuffleboard.getTab("shoot").add("P Gain", 0).getEntry();
        i = Shuffleboard.getTab("shoot").add("I Gain", 0).getEntry();
        d = Shuffleboard.getTab("shoot").add("D Gain", 0).getEntry();
        // iz = Shuffleboard.getTab("shoot").add("I Zone", 0).getEntry();
        // ff = Shuffleboard.getTab("shoot").add("Feed Forward", 0).getEntry();
        // max = Shuffleboard.getTab("shoot").add("Max Output", 0).getEntry();
        // min = Shuffleboard.getTab("shoot").add("Min Output", 0).getEntry();

        set = Shuffleboard.getTab("shoot").add("Setpoint", 0).getEntry();
    }

    // converts the velocity to RPM
    public double getVelocityRPM() {
        return m_shootEncoderRight.getVelocity() * 600 / 2048;
    }

    public void changeSetpoint() {
        
        //m_aimBot.setReference(setpoint, CANSparkMax.ControlType.kVelocity);
    }

    public void shoot() {
        m_rightShoot.set(0.5);
    }

    public void stop(){
        m_rightShoot.setIdleMode(IdleMode.kCoast);
        m_rightShoot.set(0.0);
    }

    // Takes the setpoint for the motor and makes the motor race towards the
    // setpoint
    // public void updateShoot(double setpoint){
    // final double PIDOutput = m_aimBot.calculate(getVelocityRPM(), setpoint);
    // m_rightShoot.set(PIDOutput);
    // }

    @Override
    public void periodic() {
        System.out.println(getVelocityRPM());

        P = p.getDouble(0);
        I = i.getDouble(0);
        D = d.getDouble(0);
        setpoint = set.getDouble(0);
        // double Iz = iz.getDouble(0);
        // double Ff = ff.getDouble(0);
        // double Max = max.getDouble(0);
        // double Min = min.getDouble(0);

        System.out.println("gggggggggggggggggggggggggggggg" + P);

        m_aimBot.setP(P);
        m_aimBot.setP(I);
        m_aimBot.setP(D);
        m_aimBot.setSetpoint(setpoint);
        // if PID coefficients on SmartDashboard have changed, write new values to
        // controller
        // if ((P != kP)) {
        //     m_aimBot.setP(P);
        //     kP = P;
        // }
        // if ((I != kI)) {
        //     m_aimBot.setI(I);
        //     kI = I;
        // }
        // if ((D != kD)) {
        //     m_aimBot.setD(D);
        //     kD = D;
        // }
        // if ((Iz != kIz)) {
        //     m_aimBot.setIZone(Iz);
        //     kIz = Iz;
        // }
        // if ((Ff != kFF)) {
        //     m_aimBot.setFF(Ff);
        //     kFF = Ff;
        // }
        // if ((Max != kMaxOutput) || (Min != kMinOutput)) {
        //     m_aimBot.setOutputRange(Min, Max);
        //     kMinOutput = Min;
        //     kMaxOutput = Max;
        // }
    }

}
