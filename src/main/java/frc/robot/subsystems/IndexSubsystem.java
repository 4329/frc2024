package frc.robot.subsystems;

import com.revrobotics.CANSparkBase.IdleMode;
import com.revrobotics.CANSparkMax;
import edu.wpi.first.networktables.GenericEntry;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
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

  public void bothIn() {
    backIndexMotor.set(0.2);
    frontIndexMotor.set(0.2);
  }

  public void bothSlowIn() {
    backIndexMotor.set(0.2);
    frontIndexMotor.set(0.2);
  }

  public void inShoot() {
    backIndexMotor.set(0.6);
    frontIndexMotor.set(0.6);
  }

  public void bothOut() {
    backIndexMotor.set(-0.8);
    frontIndexMotor.set(-0.8);
  }

  GenericEntry sspeed = Shuffleboard.getTab("Adsf").add("dsf", 0.4).getEntry();

  public void backInFrontOut() {
    // backIndexMotor.set(sspeed.getDouble(0)); // 0.6
    // frontIndexMotor.set(-sspeed.getDouble(0)); // -0.6
    backIndexMotor.set(0.65);
    frontIndexMotor.set(-0.55);
  }

  public void backOutFrontIn() {
    backIndexMotor.set(-0.8);
    frontIndexMotor.set(0.8);
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
    backIndexMotor.set(-0.075);
    frontIndexMotor.set(-0.075);
  }
}
