package frc.robot.commands.LightCommands;

import edu.wpi.first.wpilibj.Timer;
import frc.robot.subsystems.lightSubsystem.LightSubsystem;
import frc.robot.utilities.LightCommand;
import org.littletonrobotics.junction.Logger;

public class LightSinCommand extends LightCommand {
  private LightSubsystem lightSubsystem;
  private double offset;
  private double lastTime;
  private final double period = 4.0;
  private final double amplitude = 150;
  private final double baseLine = 0;

  private final double waveSpeed = 20.0;
  private final double pulseSpeed = 4.0;

  private final double scaleFactor = 0.8;

  public LightSinCommand(LightSubsystem lightSubsystem) {
    super(lightSubsystem);
    this.lightSubsystem = lightSubsystem;

    addRequirements(lightSubsystem);
  }

  @Override
  public void initialize() {
    super.initialize();
  }

  @Override
  public void execute() {
    super.execute();

    double[] a = new double[lightSubsystem.getLength()];
    for (int i = 0; i < lightSubsystem.getLength(); i++) {
      double initialI = ((double) i) / (double) lightSubsystem.getLength();
      double sin = Math.sin((initialI + offset) * (2 * Math.PI) * period);
      int saturation = (int) Math.round((sin + 1.0) * (amplitude / 2.0));
      saturation = (int) Math.max(saturation, baseLine);
      int pulseI =
          (int)
              Math.round(
                  (Math.sin(Timer.getFPGATimestamp() * pulseSpeed) + 1.0)
                      * ((255.0 - amplitude) / 2.0));
      lightSubsystem.setHSV(i, 105, 255, (int) ((saturation + pulseI) * scaleFactor));
    }

    Logger.recordOutput("sadff", a);

    double deltaTime = Timer.getFPGATimestamp() - lastTime;
    lastTime = Timer.getFPGATimestamp();
    deltaTime *= waveSpeed;
    offset += deltaTime / lightSubsystem.getLength();
  }

  @Override
  public boolean isFinished() {
    return false;
  }

  @Override
  public boolean runsWhenDisabled() {
    return true;
  }
}
