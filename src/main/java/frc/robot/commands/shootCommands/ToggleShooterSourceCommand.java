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
  LightSubsystem lightSubsystem;

  private GenericEntry toggleEntry;

  public ToggleShooterSourceCommand(
      ShooterSourceCommand shooterSourceCommand,
      IndexReverseForShotCommand indexReverseForShotCommand,
      ElevatorSubsystem elevatorSubsystem,
      ShootSubsystem shootSubsystem,
      ArmAngleSubsystem armAngleSubsystem,
      LightSubsystem lightSubsystem) {

    indexShooterSensorGroup = new SequentialCommandGroup(shooterSourceCommand);
    this.indexReverseForShotCommand = indexReverseForShotCommand;
    this.elevatorSubsystem = elevatorSubsystem;
    this.shootSubsystem = shootSubsystem;
    this.armAngleSubsystem = armAngleSubsystem;
    this.lightSubsystem = lightSubsystem;

    toggleEntry =
        Shuffleboard.getTab("RobotData")
            .add("Source Intake Toggled", false)
            .withPosition(3, 3)
            .withSize(3, 1)
            .withProperties(Map.of("Color when true", "#FFFF00", "Color when false", "#000000"))
            .getEntry();

    // addRequirements(elevatorSubsystem, armAngleSubsystem, shootSubsystem);
  }

  @Override
  public void initialize() {
    if (!indexShooterSensorGroup.isScheduled()) {
      indexShooterSensorGroup.schedule();
      lightSubsystem.setLEDPattern(LEDPattern.YELLOW);
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

    if (!interrupted) {
      lightSubsystem.setLEDPattern(LEDPattern.ORANGE);
    } else {
      lightSubsystem.setLEDPattern(LEDPattern.NOTHING);
    }

    new ElevatorCommand(elevatorSubsystem, ElevatorSetpoints.ZERO).schedule();
    new ArmCommand(armAngleSubsystem, ArmAngle.INTAKE).schedule();
    shootSubsystem.changeSetpoint(0);
  }
}
