package frc.robot.subsystems;

import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.utilities.SparkFactory;

public class ShootSubsystem extends SubsystemBase{

    public final CANSparkMax m_rightShoot;
    public final CANSparkMax m_leftShoot;
    public final RelativeEncoder m_shootEncoderRight;
    public final RelativeEncoder m_shootEncoderLeft;
    public final PIDController m_aimBot;
    

    //240 inches is the theroetical max shot for the shooter
    public ShootSubsystem(double P, double I, double D){
        m_rightShoot = SparkFactory.createCANSparkMax(0, false);
        m_leftShoot = SparkFactory.createCANSparkMax(1, false);
        m_aimBot = new PIDController(P, I, D);
        m_shootEncoderRight = m_rightShoot.getEncoder();
        m_shootEncoderLeft = m_leftShoot.getEncoder();
    }

    //converts the velocity to RPM
    public double getVelocityRPM(){
        return m_shootEncoderRight.getVelocity()*600/2048;
    }

    public void createPID(double P, double I, double D){
         
    }

    //Takes the setpoint for the motor and makes the motor race towards the setpoint
    public void updateShoot(double setpoint){
        final double PIDOutput = m_aimBot.calculate(getVelocityRPM(), setpoint);
        m_rightShoot.set(PIDOutput);
    }

    @Override
    public void periodic(){
        System.out.println(getVelocityRPM());
    }

    
}
