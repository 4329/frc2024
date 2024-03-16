package frc.robot.commands.driveCommands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.swerve.Drivetrain;

public class CoastCommand extends Command {

  private Drivetrain drivetrain;

  public CoastCommand(Drivetrain drivetrain) {

    this.drivetrain = drivetrain;
    addRequirements(drivetrain);
  }

  @Override
  public void initialize() {

    drivetrain.coastMode();
  }

  @Override
  public void execute() {
    drivetrain.coastMode();
  }
}
