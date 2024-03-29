package frc.robot.subsystems;

import com.revrobotics.CANSparkBase.IdleMode;
import com.revrobotics.CANSparkMax;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.Model.IndexLogAutoLogged;
import frc.robot.subsystems.LoggingSubsystem.LoggedSubsystem;
import frc.robot.utilities.SparkFactory;
import org.littletonrobotics.junction.inputs.LoggableInputs;

public class IndexSubsystem extends SubsystemBase implements LoggedSubsystem {

  private CANSparkMax topIndexMotor;
  private CANSparkMax bottomIndexMotor;
  private IndexLogAutoLogged indexLogAutoLogged;

  public IndexSubsystem() {

    topIndexMotor = SparkFactory.createCANSparkMax(14);
    bottomIndexMotor = SparkFactory.createCANSparkMax(15);
    topIndexMotor.enableVoltageCompensation(Constants.voltageCompensation);
    bottomIndexMotor.enableVoltageCompensation(Constants.voltageCompensation);
    topIndexMotor.setIdleMode(IdleMode.kBrake);
    bottomIndexMotor.setIdleMode(IdleMode.kBrake);
    indexLogAutoLogged = new IndexLogAutoLogged();

    topIndexMotor.setInverted(true);
    bottomIndexMotor.setInverted(true);

    topIndexMotor.burnFlash();
    bottomIndexMotor.burnFlash();

    // bottomIndexMotor.follow(topIndexMotor, true);

  }

  public void in() {
    topIndexMotor.set(0.8);
    bottomIndexMotor.set(0.4);
  }

  public void inForShot() {
    topIndexMotor.set(1);
    bottomIndexMotor.set(0.5);
  }

  public void out() {
    topIndexMotor.set(-0.8);
    bottomIndexMotor.set(-0.4);
  }

  public void stop() {
    topIndexMotor.set(0);
    bottomIndexMotor.set(0);
  }

  @Override
  public LoggableInputs log() {
    indexLogAutoLogged.topIndexMotor = topIndexMotor.get();
    indexLogAutoLogged.bottomIndexMotor = bottomIndexMotor.get();
    return indexLogAutoLogged;
  }

  public void slowOut() {
    topIndexMotor.set(-0.2);
    bottomIndexMotor.set(-0.1);
  }
}
