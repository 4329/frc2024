package frc.robot.commands.elevatorCommands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.ElevatorSubsystem;
import java.util.function.Supplier;

public class ElevatorManualCommand extends Command {

  private ElevatorSubsystem elevatorSubsystem;
  private Supplier<Double> rightTriggerSupplier;
  private Supplier<Double> leftTriggerSupplier;

  public ElevatorManualCommand(
      ElevatorSubsystem elevatorSubsystem,
      Supplier<Double> leftTriggerSupplier,
      Supplier<Double> rightTriggerSupplier) {

    this.elevatorSubsystem = elevatorSubsystem;
    this.leftTriggerSupplier = leftTriggerSupplier;
    this.rightTriggerSupplier = rightTriggerSupplier;
  }

  @Override
  public void initialize() {}

  @Override
  public void execute() {
    if (rightTriggerSupplier.get() > 0.1) {
      elevatorSubsystem.elevatorMove(rightTriggerSupplier.get() * 2);

    } else if (leftTriggerSupplier.get() > 0.1) {
      elevatorSubsystem.elevatorMove(-leftTriggerSupplier.get() * 2);
    } else {

      elevatorSubsystem.stop();
    }
  }

  @Override
  public boolean isFinished() {

    return false;
  }

  @Override
  public void end(boolean interrupted) {}
}
