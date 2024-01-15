package frc.robot.subsystems;

import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;

import edu.wpi.first.math.controller.PIDController;
import frc.robot.utilities.SparkFactory;

public class LinearShoot {

    public final CANSparkMax m_rightShoot;
    public final CANSparkMax m_leftShoot;
    public final RelativeEncoder m_shootEncoderRight;
    public final RelativeEncoder m_shootEncoderLeft;
    public final PIDController m_aimBot;
    

    //240 inch 
    public LinearShoot(double P, double I, double D){
        m_rightShoot = SparkFactory.createCANSparkMax(0, false);
        m_leftShoot = SparkFactory.createCANSparkMax(1, false);
        m_aimBot = new PIDController(P, I, D);
        m_shootEncoderRight = m_rightShoot.getEncoder();
        m_shootEncoderLeft = m_leftShoot.getEncoder();
    }

    public double getVelocityRPM(){
        return m_shootEncoderRight.getVelocity()*600/2048;
      }

    
    public void updateShoot(double setpoint){

        final double PIDOutput = m_aimBot.calculate(getVelocityRPM(), setpoint);
        m_rightShoot.set(PIDOutput);
    }
}
