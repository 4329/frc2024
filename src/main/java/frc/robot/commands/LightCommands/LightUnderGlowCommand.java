package frc.robot.commands.LightCommands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.lightSubsystem.LightSubsystem;

public class LightUnderGlowCommand extends Command {

  private LightSubsystem lightIndividualSubsystem;

  public LightUnderGlowCommand(LightSubsystem lightIndividualSubsystem) {
    this.lightIndividualSubsystem = lightIndividualSubsystem;
  }

  @Override
  public void execute() {
    lightIndividualSubsystem.beforeMatchColors();
  }
}
