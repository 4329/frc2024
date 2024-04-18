package frc.robot.subsystems;

import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Model.LineBreakAutoLogged;
import frc.robot.subsystems.LightSubsystem.LEDPattern;
import frc.robot.subsystems.LoggingSubsystem.LoggedSubsystem;
import org.littletonrobotics.junction.inputs.LoggableInputs;

public class LineBreakSensorSubsystem extends SubsystemBase implements LoggedSubsystem {

  private DigitalInput shooterLineBreak;
  private DigitalInput telemetryLineBreak;
  private LineBreakAutoLogged lineBreakAutoLogged;
  private LightSubsystem lightSubsystem;

  public LineBreakSensorSubsystem(LightSubsystem lightSubsystem) {

    this.shooterLineBreak = new DigitalInput(1);
    this.lightSubsystem = lightSubsystem;
    lineBreakAutoLogged = new LineBreakAutoLogged();
  }

  @Override
  public LoggableInputs log() {
    lineBreakAutoLogged.broken = !isNotBroken();
    return lineBreakAutoLogged;
  }

  public boolean isNotBroken() {
    return shooterLineBreak.get();
  }

  @Override
  public void periodic() {
    if (NoteStore.isNoted()) lightSubsystem.setLEDPattern(LEDPattern.ORANGE);
  }

  public static class NoteStore {
    private static boolean noted;

    public static boolean isNoted() {
      return noted;
    }

    public static void setNoted(boolean noted) {
      NoteStore.noted = noted;
    }
  }
}
