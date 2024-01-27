package frc.robot.subsystems;

import org.littletonrobotics.junction.Logger;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkBase.IdleMode;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Model.IndexLogAutoLogged;
import frc.robot.utilities.SparkFactory;

public class IndexSubsystem extends SubsystemBase {

    private CANSparkMax topIndexMotor;
    private CANSparkMax bottomIndexMotor;
    private IndexLogAutoLogged indexLogAutoLogged;

    public IndexSubsystem() {

        topIndexMotor = SparkFactory.createCANSparkMax(14);
        bottomIndexMotor = SparkFactory.createCANSparkMax(15);
        topIndexMotor.setIdleMode(IdleMode.kBrake);
        bottomIndexMotor.setIdleMode(IdleMode.kBrake);
        indexLogAutoLogged = new IndexLogAutoLogged();

        // bottomIndexMotor.follow(topIndexMotor, true);

    }

    public void in() {
        topIndexMotor.set(0.3);
        bottomIndexMotor.set(-0.5);

    }

    public void out() {
        topIndexMotor.set(-0.3);
        bottomIndexMotor.set(0.5);

    }

    public void stop() {
        topIndexMotor.set(0);
        bottomIndexMotor.set(0);

    }
    
    @Override
    public void periodic() {
        indexLogAutoLogged.topIndexMotor = topIndexMotor.get();
        indexLogAutoLogged.bottomIndexMotor = bottomIndexMotor.get();
        Logger.processInputs("index", indexLogAutoLogged);
    }

}
