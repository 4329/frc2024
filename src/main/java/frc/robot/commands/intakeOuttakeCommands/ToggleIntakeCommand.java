package frc.robot.commands.intakeOuttakeCommands;

import edu.wpi.first.networktables.GenericEntry;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.commands.armCommands.ArmAngleCommand;
import frc.robot.commands.elevatorCommands.ElevatorCommand;
import frc.robot.commands.indexCommands.IndexReverseForShotCommand;
import frc.robot.commands.indexCommands.IndexSensorCommand;
import frc.robot.subsystems.ArmAngleSubsystem;
import frc.robot.subsystems.ElevatorSubsystem;
import frc.robot.subsystems.LightSubsystem;
import frc.robot.subsystems.LightSubsystem.LEDPattern;
import frc.robot.subsystems.ShootSubsystem;
import frc.robot.utilities.ArmAngle;
import frc.robot.utilities.ElevatorSetpoints;
import frc.robot.utilities.ReInitCommand;
import java.util.Map;

public class ToggleIntakeCommand extends ReInitCommand {
  SequentialCommandGroup intakeSensorGroup;
  IndexReverseForShotCommand indexReverseForShotCommand;
  ElevatorSubsystem elevatorSubsystem;
  LightSubsystem lightSubsystem;
  ShootSubsystem shootSubsystem;

  private GenericEntry toggleEntry;

  public ToggleIntakeCommand(
      IntakeSensorCommand intakeSensorCommand,
      IndexSensorCommand indexSensorCommand,
      IndexReverseForShotCommand indexReverseForShotCommand,
      ElevatorSubsystem elevatorSubsystem,
      ArmAngleSubsystem armAngleSubsystem,
      LightSubsystem lightSubsystem,
      ShootSubsystem shootSubsystem) {
    intakeSensorGroup =
        intakeSensorCommand
            .alongWith(indexSensorCommand)
            .beforeStarting(
                new ArmAngleCommand(armAngleSubsystem, ArmAngle.INTAKE)
                    .beforeStarting(
                        new ElevatorCommand(elevatorSubsystem, ElevatorSetpoints.ZERO)));
    this.indexReverseForShotCommand = indexReverseForShotCommand;
    this.elevatorSubsystem = elevatorSubsystem;
    this.lightSubsystem = lightSubsystem;
    this.shootSubsystem = shootSubsystem;

    toggleEntry =
        Shuffleboard.getTab("RobotData")
            .add("Intake Toggled", false)
            .withPosition(10, 3)
            .withSize(3, 2)
            .withProperties(Map.of("Color when true", "#00FF00", "Color when false", "#000000"))
            .getEntry();
  }

  @Override
  public void initialize() {
    if (!intakeSensorGroup.isScheduled()) {
      intakeSensorGroup.schedule();
      lightSubsystem.setLEDPattern(LEDPattern.GREEN);
    } else {
      this.cancel();
    }

    toggleEntry.setBoolean(intakeSensorGroup.isScheduled());
  }

  @Override
  public boolean isFinished() {
    return !intakeSensorGroup.isScheduled();
  }

  @Override
  public void end(boolean interrupted) {
    shootSubsystem.changeSetpoint(0);
    intakeSensorGroup.cancel();
    toggleEntry.setBoolean(intakeSensorGroup.isScheduled());

    if (!interrupted) {
      indexReverseForShotCommand.schedule();
      lightSubsystem.setLEDPattern(LEDPattern.ORANGE);
    } else {
      lightSubsystem.setLEDPattern(LEDPattern.NOTHING);
    }

    new ElevatorCommand(elevatorSubsystem, ElevatorSetpoints.ZERO).schedule();
  }
}
