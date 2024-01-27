package frc.robot.subsystems;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkBase.IdleMode;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.utilities.SparkFactory;

public class IntakeSubsystem extends SubsystemBase {

    private CANSparkMax topIntakeMotor;
    private CANSparkMax bottomIntakeMotor;

    public IntakeSubsystem() {

        topIntakeMotor = SparkFactory.createCANSparkMax(9);
        bottomIntakeMotor = SparkFactory.createCANSparkMax(10);
        topIntakeMotor.setIdleMode(IdleMode.kBrake);
        bottomIntakeMotor.setIdleMode(IdleMode.kBrake);

        bottomIntakeMotor.follow(topIntakeMotor, true);

    }

    public void in() {
        topIntakeMotor.set(0.1);

    }

    public void out() {
        topIntakeMotor.set(-0.1);

    }

    public void stop() {
        topIntakeMotor.set(0);

    }

}
