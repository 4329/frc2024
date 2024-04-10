package frc.robot.commands.shootCommands;

import edu.wpi.first.networktables.GenericEntry;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.commands.armCommands.ArmCommand;
import frc.robot.commands.elevatorCommands.ElevatorCommand;
import frc.robot.commands.indexCommands.IndexReverseForShotCommand;
import frc.robot.subsystems.ArmAngleSubsystem;
import frc.robot.subsystems.ElevatorSubsystem;
import frc.robot.subsystems.ShootSubsystem;
import frc.robot.utilities.ArmAngle;
import frc.robot.utilities.ElevatorSetpoints;
import frc.robot.utilities.ReInitCommand;
import java.util.Map;

public class ToggleShooterSourceCommand extends ReInitCommand {
  SequentialCommandGroup indexShooterSensorGroup;
  IndexReverseForShotCommand indexReverseForShotCommand;
  ShootSubsystem shootSubsystem;
  ElevatorSubsystem elevatorSubsystem;
  ArmAngleSubsystem armAngleSubsystem;

  // private boolean toggled;
  private GenericEntry toggleEntry;

  public ToggleShooterSourceCommand(
      ShooterSourceCommand shooterSourceCommand,
      IndexReverseForShotCommand indexReverseForShotCommand,
      ElevatorSubsystem elevatorSubsystem,
      ShootSubsystem shootSubsystem,
      ArmAngleSubsystem armAngleSubsystem) {

    indexShooterSensorGroup = new SequentialCommandGroup(shooterSourceCommand);
    // .alongWith(shooterSourceCommand);
    // .beforeStarting(new ArmAngleCommand(armAngleSubsystem, ArmAngle.SHOOTERSOURCE));
    this.indexReverseForShotCommand = indexReverseForShotCommand;
    this.elevatorSubsystem = elevatorSubsystem;
    this.shootSubsystem = shootSubsystem;
    this.armAngleSubsystem = armAngleSubsystem;

    toggleEntry =
        Shuffleboard.getTab("RobotData")
            .add("Source Intake Toggled", false)
            .withPosition(3, 3)
            .withSize(3, 1)
            .withProperties(Map.of("Color when true", "#FFFF00", "Color when false", "#000000"))
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
    new ArmCommand(armAngleSubsystem, ArmAngle.INTAKE).schedule();
    shootSubsystem.changeSetpoint(0);
  }
}
