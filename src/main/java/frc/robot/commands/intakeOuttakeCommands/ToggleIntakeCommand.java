package frc.robot.commands.intakeOuttakeCommands;

import edu.wpi.first.networktables.GenericEntry;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.commands.armCommands.ArmAngleCommand;
import frc.robot.commands.indexCommands.IndexReverseForShotCommand;
import frc.robot.commands.indexCommands.IndexSensorCommand;
import frc.robot.subsystems.ArmAngleSubsystem;
import frc.robot.utilities.ArmAngle;
import frc.robot.utilities.ReInitCommand;
import java.util.Map;

public class ToggleIntakeCommand extends ReInitCommand {
  SequentialCommandGroup intakeSensorGroup;
  IndexReverseForShotCommand indexReverseForShotCommand;

  // private boolean toggled;
  private GenericEntry toggleEntry;

  public ToggleIntakeCommand(
      IntakeSensorCommand intakeSensorCommand,
      IndexSensorCommand indexSensorCommand,
      IndexReverseForShotCommand indexReverseForShotCommand,
      ArmAngleSubsystem armAngleSubsystem) {
    intakeSensorGroup =
        intakeSensorCommand
            .alongWith(indexSensorCommand)
            .beforeStarting(new ArmAngleCommand(armAngleSubsystem, ArmAngle.INTAKE));
    this.indexReverseForShotCommand = indexReverseForShotCommand;

    toggleEntry =
        Shuffleboard.getTab("RobotData")
            .add("Intake Toggled", false)
            .withPosition(3, 5)
            .withSize(10, 1)
            .withProperties(Map.of("Color when true", "#00FF00", "Color when false", "#000000"))
            .getEntry();
  }

  @Override
  public void initialize() {
    if (!intakeSensorGroup.isScheduled()) {
      intakeSensorGroup.schedule();
    } else this.cancel();
    // toggled = !toggled;
    toggleEntry.setBoolean(intakeSensorGroup.isScheduled());
  }

  @Override
  public void execute() {
    System.out.println(intakeSensorGroup.isScheduled());
  }

  @Override
  public boolean isFinished() {
    return !intakeSensorGroup.isScheduled();
  }

  @Override
  public void end(boolean interrupted) {
    intakeSensorGroup.cancel();
    // toggled = false;
    toggleEntry.setBoolean(intakeSensorGroup.isScheduled());
    System.out.println(
        intakeSensorGroup.isScheduled()
            + "{{{{{{{{{{{{{{{{{{{{{{{{{{{{{{{{{{{{{}}}}}}}}}}}}}}}}}}}}}}}}}}}}}}}}}}}}}");
    if (!interrupted) {
      indexReverseForShotCommand.schedule();
    }
  }
}
