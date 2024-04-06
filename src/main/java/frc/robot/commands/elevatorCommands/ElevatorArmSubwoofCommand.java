package frc.robot.commands.elevatorCommands;

import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import frc.robot.commands.armCommands.ArmAngleCommand;
import frc.robot.subsystems.ArmAngleSubsystem;
import frc.robot.subsystems.ElevatorSubsystem;
import frc.robot.utilities.ArmAngle;
import frc.robot.utilities.ElevatorSetpoints;

public class ElevatorArmSubwoofCommand extends ParallelCommandGroup {

  public ElevatorArmSubwoofCommand(
      ElevatorSubsystem elevatorSubsystem, ArmAngleSubsystem armAngleSubsystem) {

    super(
        new ElevatorCommand(elevatorSubsystem, ElevatorSetpoints.SUBWOOF),
        new ArmAngleCommand(armAngleSubsystem, ArmAngle.SUBWOOF));
  }
}
