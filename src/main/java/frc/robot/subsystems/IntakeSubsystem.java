package frc.robot.subsystems;

import com.revrobotics.CANSparkMax;

import org.littletonrobotics.junction.Logger;

import com.revrobotics.CANSparkBase.IdleMode;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Model.IntakeLogAutoLogged;
import frc.robot.utilities.SparkFactory;

public class IntakeSubsystem extends SubsystemBase {

    private CANSparkMax topIntakeMotor;
    private CANSparkMax bottomIntakeMotor;
    public IntakeLogAutoLogged intakeLogAutoLogged;

    public IntakeSubsystem() {

        topIntakeMotor = SparkFactory.createCANSparkMax(9);
        bottomIntakeMotor = SparkFactory.createCANSparkMax(10);
        topIntakeMotor.setIdleMode(IdleMode.kBrake);
        bottomIntakeMotor.setIdleMode(IdleMode.kBrake);
        intakeLogAutoLogged = new IntakeLogAutoLogged();

        bottomIntakeMotor.follow(topIntakeMotor, false);

        topIntakeMotor.burnFlash();
        bottomIntakeMotor.burnFlash();

    }

    public void in() {
        topIntakeMotor.set(0.5);

    }

    public void out() {
        topIntakeMotor.set(-0.5);

    }

    public void stop() {
        topIntakeMotor.set(0);

    }
    @Override
    public void periodic() {
        intakeLogAutoLogged.topIntakeMotor = topIntakeMotor.get() != 0;
        Logger.processInputs("Intaking", intakeLogAutoLogged);
    }

}
