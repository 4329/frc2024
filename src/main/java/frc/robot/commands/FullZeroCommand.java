package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.commands.armCommands.ArmCommand;
import frc.robot.commands.elevatorCommands.ElevatorCommand;
import frc.robot.subsystems.ArmAngleSubsystem;
import frc.robot.subsystems.ElevatorSubsystem;
import frc.robot.utilities.ArmAngle;
import frc.robot.utilities.ElevatorSetpoints;

public class FullZeroCommand extends SequentialCommandGroup {

  public FullZeroCommand(ElevatorSubsystem elevatorSubsystem, ArmAngleSubsystem armAngleSubsystem) {

    super(
        new ElevatorCommand(elevatorSubsystem, ElevatorSetpoints.ZERO),
        new ArmCommand(armAngleSubsystem, ArmAngle.ZERO));
  }
}
