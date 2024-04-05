package frc.robot.commands.indexCommands;

import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.subsystems.ArmAngleSubsystem;
import frc.robot.subsystems.IndexSubsystem;
import frc.robot.subsystems.LineBreakSensorSubsystem;

public class IndexSourceSensorCommand extends SequentialCommandGroup {

  public IndexSourceSensorCommand(
      IndexSubsystem indexSubsystem,
      ArmAngleSubsystem armAngleSubsystem,
      LineBreakSensorSubsystem lineBreakSensorSubsystem) {

    super(
        // new ArmAngleCommand(armAngleSubsystem, ArmAngle.INDEXSOURCE),
        new IndexSourceCommand(lineBreakSensorSubsystem, indexSubsystem));
  }
}
