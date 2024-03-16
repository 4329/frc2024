package frc.robot.commands.LightCommands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.lightSubsystem.LightSubsystem;

public class LightBlackCommand extends Command {
  public LightSubsystem lightIndividualSubsystem;

  public LightBlackCommand(LightSubsystem lightIndividualSubsystem) {
    this.lightIndividualSubsystem = lightIndividualSubsystem;
    addRequirements(lightIndividualSubsystem);
  }

  @Override
  public void initialize() {
    System.out.println(
        "d____________________________________________________________________________________________________________________jidljldjkjlkjjkjdlkjkldjlkjdjdkljd");
    lightIndividualSubsystem.setBlack();
  }

  @Override
  public void execute() {}

  @Override
  public boolean isFinished() {
    return false;
  }

  @Override
  public void end(boolean interrupted) {
    System.out.println("sdfaijlfidjsalifjsdlijfsadlifjdsilifjodsjafjiodj");
  }
}
