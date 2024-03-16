package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.lightSubsystem.LightSubsystem;

public class BeforeMatchCommand extends Command {

  public LightSubsystem lightIndividualSubsystem;

  public BeforeMatchCommand(LightSubsystem lightIndividualSubsystem) {
    this.lightIndividualSubsystem = lightIndividualSubsystem;
    addRequirements(lightIndividualSubsystem);
  }

  @Override
  public void execute() {
    lightIndividualSubsystem.beforeMatchColors();
  }

  @Override
  public boolean runsWhenDisabled() {
    return true;
  }
}
