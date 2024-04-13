package frc.robot.commands.climberCommands;

import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import frc.robot.commands.armCommands.ArmAngleCommand;
import frc.robot.subsystems.ArmAngleSubsystem;
import frc.robot.subsystems.ClimberSubsystem;
import frc.robot.utilities.ArmAngle;
import frc.robot.utilities.UnInstantCommand;

public class ClimberClimbCommand extends ParallelCommandGroup {

  public ClimberClimbCommand(
      ArmAngleSubsystem armAngleSubsystem, ClimberSubsystem climberSubsystem) {

    super(
        new ArmAngleCommand(armAngleSubsystem, ArmAngle.CLIMB),
        new UnInstantCommand(() -> climberSubsystem.toggleTarget()));
  }
}
