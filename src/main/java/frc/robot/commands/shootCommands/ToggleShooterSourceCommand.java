package frc.robot.commands.shootCommands;

import edu.wpi.first.networktables.GenericEntry;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.commands.armCommands.ArmCommand;
import frc.robot.commands.elevatorCommands.ElevatorCommand;
import frc.robot.commands.indexCommands.IndexReverseForShotCommand;
import frc.robot.subsystems.ArmAngleSubsystem;
import frc.robot.subsystems.ElevatorSubsystem;
import frc.robot.subsystems.LightSubsystem;
import frc.robot.subsystems.LightSubsystem.LEDPattern;
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

  private GenericEntry toggleEntry;

  public ToggleShooterSourceCommand(
      ShooterSourceCommand shooterSourceCommand,
      IndexReverseForShotCommand indexReverseForShotCommand,
      ElevatorSubsystem elevatorSubsystem,
      ShootSubsystem shootSubsystem,
      ArmAngleSubsystem armAngleSubsystem) {

    indexShooterSensorGroup = shooterSourceCommand;
    this.indexReverseForShotCommand = indexReverseForShotCommand;
    this.elevatorSubsystem = elevatorSubsystem;
    this.shootSubsystem = shootSubsystem;
    this.armAngleSubsystem = armAngleSubsystem;

    toggleEntry =
        Shuffleboard.getTab("RobotData")
            .add("Source Intake Toggled", false)
            .withPosition(4, 3)
            .withSize(3, 2)
            .withProperties(Map.of("Color when true", "#FFFF00", "Color when false", "#000000"))
            .getEntry();

    // addRequirements(elevatorSubsystem, armAngleSubsystem, shootSubsystem);
  }

  @Override
  public void initialize() {
    if (!indexShooterSensorGroup.isScheduled()) {
      indexShooterSensorGroup.schedule();
      LightSubsystem.State.setSourcing(true);
    } else {
      this.cancel();
    }

    toggleEntry.setBoolean(indexShooterSensorGroup.isScheduled());
  }

  @Override
  public boolean isFinished() {
    return !indexShooterSensorGroup.isScheduled();
  }

  @Override
  public void end(boolean interrupted) {
    indexShooterSensorGroup.cancel();
    toggleEntry.setBoolean(indexShooterSensorGroup.isScheduled());

    LightSubsystem.State.setHasNote(!interrupted);
    LightSubsystem.State.setSourcing(false);

    new ElevatorCommand(elevatorSubsystem, ElevatorSetpoints.ZERO).schedule();
    new ArmCommand(armAngleSubsystem, ArmAngle.INTAKE).schedule();
    shootSubsystem.changeSetpoint(0);
  }
}
