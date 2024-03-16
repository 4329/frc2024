package frc.robot.commands.LightCommands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.lightSubsystem.LightSubsystem;

public class LightsOnCommand extends Command {
  LightSubsystem lightIndividualSubsystem;

  public LightsOnCommand(LightSubsystem lightIndividualSubsystem) {
    this.lightIndividualSubsystem = lightIndividualSubsystem;
    addRequirements(lightIndividualSubsystem);
  }

  @Override
  public void initialize() {
    lightIndividualSubsystem.lightsOn();
  }

  @Override
  public boolean isFinished() {
    return false;
  }
}
