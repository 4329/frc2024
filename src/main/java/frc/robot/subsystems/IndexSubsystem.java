package frc.robot.subsystems;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkBase.IdleMode;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.utilities.SparkFactory;

public class IndexSubsystem extends SubsystemBase {

    private CANSparkMax topIndexMotor;
    private CANSparkMax bottomIndexMotor;

    public IndexSubsystem() {

        topIndexMotor = SparkFactory.createCANSparkMax(14);
        bottomIndexMotor = SparkFactory.createCANSparkMax(15);
        topIndexMotor.setIdleMode(IdleMode.kBrake);
        bottomIndexMotor.setIdleMode(IdleMode.kBrake);

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

}
