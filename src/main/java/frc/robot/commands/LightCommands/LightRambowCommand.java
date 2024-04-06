package frc.robot.commands.LightCommands;

import edu.wpi.first.wpilibj.Timer;
import frc.robot.subsystems.lightSubsystem.LightSubsystem;
import frc.robot.utilities.LightCommand;

public class LightRambowCommand extends LightCommand {
  public LightSubsystem lightSubsystem;

  private double hue;
  private double deltaTime;
  private double lastTime;
  private double startTime;

  public LightRambowCommand(LightSubsystem lightSubsystem) {
    super(lightSubsystem);
    this.lightSubsystem = lightSubsystem;

    // addRequirements(lightIndividualSubsystem);
  }

  @Override
  public void lightInitialize() {
    hue = 0;
    startTime = Timer.getFPGATimestamp();
  }

  @Override
  public void lightExecute() {
    // For every pixel
    for (var i = 0; i < lightSubsystem.getLength(); i++) {
      // Calculate the hue - hue is easier for rainbows because the color
      // shape is a circle so only one value needs to precess
      final int hues = (int) (hue + (i * 180 / lightSubsystem.getLength())) % 180;
      // Set the value
      lightSubsystem.setHSV(i, hues, 255, 128);
    }
    // Increase by to make the rainbow "move"
    deltaTime = Timer.getFPGATimestamp() - lastTime;
    lastTime = Timer.getFPGATimestamp();
    hue += deltaTime * 40;
    // hue++;
    // System.out.println(Math.random() * 20);
    // Check bounds
    hue %= 180;
  }

  @Override
  public boolean isFinished() {
    return false;
  }
}
