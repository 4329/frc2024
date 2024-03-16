package frc.robot.commands.LightCommands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.lightSubsystem.LightSubsystem;

public class LightRambowCommand extends Command {
  public LightSubsystem lightIndividualSubsystem;

  public LightRambowCommand(LightSubsystem lightIndividualSubsystem) {
    this.lightIndividualSubsystem = lightIndividualSubsystem;

    addRequirements(lightIndividualSubsystem);
  }

  @Override
  public void execute() {
    lightIndividualSubsystem.rainbow();
    System.out.println("rainbow+++++++++++++++++++");
  }

  @Override
  public boolean isFinished() {
    return false;
  }

  @Override
  public void end(boolean interrupted) {
    lightIndividualSubsystem.beforeMatchColors();
  }
}
