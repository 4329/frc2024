package frc.robot.commands.indexCommands;

import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.subsystems.ArmAngleSubsystem;
import frc.robot.subsystems.IndexSubsystem;

public class AmpOutDexCommand extends SequentialCommandGroup {

  public AmpOutDexCommand(IndexSubsystem indexSubsystem, ArmAngleSubsystem armAngleSubsystem) {

    super(
        // new ArmAngleCommand(armAngleSubsystem, ArmAngle.AMPDEX),
        new IndexAmpCommand(indexSubsystem).withTimeout(3));
  }
}
