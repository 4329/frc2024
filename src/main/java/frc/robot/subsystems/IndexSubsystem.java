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

  private CANSparkMax backIndexMotor;
  private CANSparkMax frontIndexMotor;
  private IndexLogAutoLogged indexLogAutoLogged;

  public IndexSubsystem() {

    backIndexMotor = SparkFactory.createCANSparkMax(14);
    frontIndexMotor = SparkFactory.createCANSparkMax(15);
    backIndexMotor.enableVoltageCompensation(Constants.voltageCompensation);
    frontIndexMotor.enableVoltageCompensation(Constants.voltageCompensation);
    backIndexMotor.setIdleMode(IdleMode.kBrake);
    frontIndexMotor.setIdleMode(IdleMode.kBrake);
    indexLogAutoLogged = new IndexLogAutoLogged();

    backIndexMotor.setInverted(true);
    frontIndexMotor.setInverted(true);

    backIndexMotor.burnFlash();
    frontIndexMotor.burnFlash();

    // bottomIndexMotor.follow(topIndexMotor, true);

  }

  public void in() {
    backIndexMotor.set(0.8);
    frontIndexMotor.set(0.8);
  }

  public void inForShot() {
    backIndexMotor.set(1);
    frontIndexMotor.set(0.1);
  }

  public void out() {
    backIndexMotor.set(-0.8);
    frontIndexMotor.set(-0.8);
  }

  public void ampOut() {
    backIndexMotor.set(1);
    frontIndexMotor.set(-1);
  }

  public void stop() {
    backIndexMotor.set(0);
    frontIndexMotor.set(0);
  }

  @Override
  public LoggableInputs log() {
    indexLogAutoLogged.topIndexMotor = backIndexMotor.get();
    indexLogAutoLogged.bottomIndexMotor = frontIndexMotor.get();
    return indexLogAutoLogged;
  }

  public void slowOut() {
    backIndexMotor.set(-0.2);
    frontIndexMotor.set(-0.1);
  }
}
