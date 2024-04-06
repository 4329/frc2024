package frc.robot.commands.armCommands;

import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import frc.robot.commands.elevatorCommands.ElevatorCommand;
import frc.robot.subsystems.ArmAngleSubsystem;
import frc.robot.subsystems.ElevatorSubsystem;
import frc.robot.utilities.ArmAngle;
import frc.robot.utilities.ElevatorSetpoints;

public class ArmElevatorSourceCommand extends ParallelCommandGroup {

  public ArmElevatorSourceCommand(
      ArmAngleSubsystem armAngleSubsystem, ElevatorSubsystem elevatorSubsystem) {

    super(
        new ElevatorCommand(elevatorSubsystem, ElevatorSetpoints.SOURCE),
        new ArmAngleCommand(armAngleSubsystem, ArmAngle.SHOOTERSOURCE));
  }
}
