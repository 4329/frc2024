package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.commands.armCommands.ArmCommand;
import frc.robot.commands.elevatorCommands.ElevatorCommand;
import frc.robot.subsystems.ArmAngleSubsystem;
import frc.robot.subsystems.ElevatorSubsystem;
import frc.robot.utilities.ArmAngle;
import frc.robot.utilities.ElevatorSetpoints;

public class ArmToHorizontalComand extends SequentialCommandGroup {
    
  public ArmToHorizontalComand(
      ArmAngleSubsystem armAngleSubsystem, ElevatorSubsystem elevatorSubsystem) {
    super(
        new ElevatorCommand(elevatorSubsystem, ElevatorSetpoints.ZERO).withTimeout(3),
        new ArmCommand(armAngleSubsystem, ArmAngle.HORIZONTAL));
  }
}
