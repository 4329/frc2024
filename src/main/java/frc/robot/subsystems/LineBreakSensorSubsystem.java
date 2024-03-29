package frc.robot.subsystems;

import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Model.LineBreakAutoLogged;
import frc.robot.subsystems.LoggingSubsystem.LoggedSubsystem;
import org.littletonrobotics.junction.inputs.LoggableInputs;

public class LineBreakSensorSubsystem extends SubsystemBase implements LoggedSubsystem {

  private DigitalInput lineBreak;
  private LineBreakAutoLogged lineBreakAutoLogged;

  public LineBreakSensorSubsystem() {

    this.lineBreak = new DigitalInput(1);
    lineBreakAutoLogged = new LineBreakAutoLogged();
  }

  @Override
  public LoggableInputs log() {

    lineBreakAutoLogged.broken = !isNotBroken();
    return lineBreakAutoLogged;
  }

  public Boolean isNotBroken() {

    return lineBreak.get();
  }
}
