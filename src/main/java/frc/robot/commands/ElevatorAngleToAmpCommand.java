package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.commands.armCommands.ArmCommand;
import frc.robot.commands.elevatorCommands.ElevatorToAmpCommand;
import frc.robot.subsystems.ArmAngleSubsystem;
import frc.robot.subsystems.ElevatorSubsystem;
import frc.robot.subsystems.IndexSubsystem;
import frc.robot.subsystems.ShootSubsystem;
import frc.robot.utilities.ArmAngle;

public class ElevatorAngleToAmpCommand extends SequentialCommandGroup {
    
  public ElevatorAngleToAmpCommand(
      ShootSubsystem shootSubsystem,
      IndexSubsystem indexSubsystem,
      ArmAngleSubsystem armAngleSubsystem,
      ElevatorSubsystem elevatorSubsystem) {
    super(
        new ParallelCommandGroup(
            new ElevatorToAmpCommand(elevatorSubsystem),
            new ArmCommand(armAngleSubsystem, ArmAngle.ARMAMP)));
  }
}
