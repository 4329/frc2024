package frc.robot.subsystems;

import com.revrobotics.CANSparkBase.IdleMode;
import com.revrobotics.CANSparkMax;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.Model.IntakeLogAutoLogged;
import frc.robot.subsystems.LoggingSubsystem.LoggedSubsystem;
import frc.robot.utilities.SparkFactory;
import org.littletonrobotics.junction.inputs.LoggableInputs;

public class IntakeSubsystem extends SubsystemBase implements LoggedSubsystem {

  private CANSparkMax topIntakeMotor;
  private CANSparkMax bottomIntakeMotor;
  public IntakeLogAutoLogged intakeLogAutoLogged;

  public IntakeSubsystem() {

    topIntakeMotor = SparkFactory.createCANSparkMax(9);
    bottomIntakeMotor = SparkFactory.createCANSparkMax(10);
    topIntakeMotor.setIdleMode(IdleMode.kBrake);
    bottomIntakeMotor.setIdleMode(IdleMode.kBrake);
    intakeLogAutoLogged = new IntakeLogAutoLogged();
    topIntakeMotor.enableVoltageCompensation(Constants.voltageCompensation);
    bottomIntakeMotor.enableVoltageCompensation(Constants.voltageCompensation);

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
  public LoggableInputs log() {
    intakeLogAutoLogged.topIntakeMotor = topIntakeMotor.get() != 0;
    return intakeLogAutoLogged;
  }
}
