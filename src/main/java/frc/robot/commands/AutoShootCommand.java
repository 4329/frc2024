package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.ParallelRaceGroup;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import frc.robot.commands.armCommands.ArmCommand;
import frc.robot.commands.elevatorCommands.ElevatorCommand;
// import frc.robot.commands.elevatorCommands.ElevatorShootCommand;
import frc.robot.commands.elevatorCommands.ElevatorShootCommandIndefinite;
import frc.robot.commands.shootCommands.ShooterAimCommand;
import frc.robot.commands.shootCommands.ShooterAimCommandIndefinite;
import frc.robot.commands.shootCommands.ShooterShotCommand;
import frc.robot.commands.shootCommands.ShotRevCommand;
import frc.robot.subsystems.ArmAngleSubsystem;
import frc.robot.subsystems.ElevatorSubsystem;
import frc.robot.subsystems.IndexSubsystem;
import frc.robot.subsystems.ShootSubsystem;
import frc.robot.subsystems.VisionSubsystem;
import frc.robot.utilities.ArmAngle;
import frc.robot.utilities.ElevatorSetpoints;

public class AutoShootCommand extends SequentialCommandGroup {
  public AutoShootCommand(
      ShootSubsystem shootSubsystem,
      IndexSubsystem indexSubsystem,
      VisionSubsystem visionSubsystem,
      CommandXboxController commandXboxController,
      ElevatorSubsystem elevatorSubsystem,
      ArmAngleSubsystem armAngleSubsystem) {

    super(
        new ParallelRaceGroup(
            new ParallelCommandGroup(
                new ShooterAimCommand(visionSubsystem, armAngleSubsystem),
                // new ElevatorShootCommand(elevatorSubsystem, visionSubsystem)),
                new ShotRevCommand(shootSubsystem, visionSubsystem))),
        new ParallelRaceGroup(
            new ShooterShotCommand(shootSubsystem, indexSubsystem, visionSubsystem).withTimeout(2),
            new ShooterAimCommandIndefinite(visionSubsystem, armAngleSubsystem, elevatorSubsystem),
            new ElevatorShootCommandIndefinite(elevatorSubsystem, visionSubsystem)),
        new ParallelCommandGroup(
            new ArmCommand(armAngleSubsystem, ArmAngle.INTAKE),
            new ElevatorCommand(elevatorSubsystem, ElevatorSetpoints.ZERO)));
  }
}
