package frc.robot.commands.shootCommands;

import java.util.Map;

import edu.wpi.first.networktables.GenericEntry;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.commands.armCommands.ArmAngleCommand;
import frc.robot.commands.elevatorCommands.ElevatorCommand;
import frc.robot.commands.indexCommands.IndexReverseForShotCommand;
import frc.robot.commands.indexCommands.OutdexSensorCommand;
import frc.robot.subsystems.ArmAngleSubsystem;
import frc.robot.subsystems.ElevatorSubsystem;
import frc.robot.utilities.ArmAngle;
import frc.robot.utilities.ElevatorSetpoints;
import frc.robot.utilities.ReInitCommand;

public class ToggleShooterSourceCommand extends ReInitCommand {
  SequentialCommandGroup indexShooterSensorGroup;
  IndexReverseForShotCommand indexReverseForShotCommand;
  ElevatorSubsystem elevatorSubsystem;

  // private boolean toggled;
  private GenericEntry toggleEntry;

  public ToggleShooterSourceCommand(
      ShooterSourceCommand shooterSourceCommand,
      IndexReverseForShotCommand indexReverseForShotCommand,
      ElevatorSubsystem elevatorSubsystem,
      ArmAngleSubsystem armAngleSubsystem) {

    indexShooterSensorGroup =
        new SequentialCommandGroup()
            .alongWith(shooterSourceCommand)
            .beforeStarting(new ArmAngleCommand(armAngleSubsystem, ArmAngle.SHOOTERSOURCE));
    this.indexReverseForShotCommand = indexReverseForShotCommand;
    this.elevatorSubsystem = elevatorSubsystem;

    toggleEntry =
        Shuffleboard.getTab("RobotData")
            .add("Shoot Intake Toggled", false)
            .withPosition(3, 4)
            .withSize(10, 1)
            .withProperties(Map.of("Color when true", "#00FF00", "Color when false", "#000000"))
            .getEntry();
  }

  @Override
  public void initialize() {
    if (!indexShooterSensorGroup.isScheduled()) {
      indexShooterSensorGroup.schedule();
    } else this.cancel();
    // toggled = !toggled;
    toggleEntry.setBoolean(indexShooterSensorGroup.isScheduled());
  }

  @Override
  public void execute() {
    System.out.println(indexShooterSensorGroup.isScheduled());
  }

  @Override
  public boolean isFinished() {
    return !indexShooterSensorGroup.isScheduled();
  }

  @Override
  public void end(boolean interrupted) {
    indexShooterSensorGroup.cancel();
    // toggled = false;
    toggleEntry.setBoolean(indexShooterSensorGroup.isScheduled());
   

    new ElevatorCommand(elevatorSubsystem, ElevatorSetpoints.ZERO).schedule();
  }
}
