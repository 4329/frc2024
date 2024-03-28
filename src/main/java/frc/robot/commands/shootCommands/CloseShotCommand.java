package frc.robot.commands.shootCommands;

import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import frc.robot.commands.armCommands.ArmCommand;
import frc.robot.subsystems.ArmAngleSubsystem;
import frc.robot.subsystems.IndexSubsystem;
import frc.robot.subsystems.ShootSubsystem;
import frc.robot.utilities.ArmAngle;

public class CloseShotCommand extends ParallelCommandGroup {
  public CloseShotCommand(
      ArmAngleSubsystem angleSubsystem,
      ShootSubsystem shootSubsystem,
      IndexSubsystem indexSubsystem) {
    super(
        new AutoSideShotCommand(shootSubsystem, indexSubsystem),
        new ArmCommand(angleSubsystem, ArmAngle.ZERO));
  }
}
