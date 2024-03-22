package frc.robot.commands.shootCommands;

import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import frc.robot.commands.indexCommands.IndexCommand;
import frc.robot.subsystems.IndexSubsystem;
import frc.robot.subsystems.ShootSubsystem;

public class ReleaseToShoot extends ParallelCommandGroup {
    
  public ReleaseToShoot(
      ShootSubsystem shootSubsystem, IndexSubsystem indexSubsystem) {

    super(
        new IndexCommand(indexSubsystem).withTimeout(3),
        new ShootCommand(shootSubsystem).withTimeout(3));
  }
}
