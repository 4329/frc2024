package frc.robot.commands.climberCommands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.ClimberSubsystem;
import java.util.function.Supplier;

public class ClimberManualCommand extends Command {

  private ClimberSubsystem climberSubsystem;
  private Supplier<Double> rightTriggerSupplier;
  private Supplier<Double> leftTriggerSupplier;

  public ClimberManualCommand(
      ClimberSubsystem climberSubsystem,
      Supplier<Double> leftTriggerSupplier,
      Supplier<Double> rightTriggerSupplier) {

    this.climberSubsystem = climberSubsystem;
    this.leftTriggerSupplier = leftTriggerSupplier;
    this.rightTriggerSupplier = rightTriggerSupplier;
  }

  @Override
  public void initialize() {}

  @Override
  public void execute() {
    if (rightTriggerSupplier.get() > 0.1) {
      climberSubsystem.climberMove(rightTriggerSupplier.get() * 2);

    } else if (leftTriggerSupplier.get() > 0.1) {
      climberSubsystem.climberMove(-leftTriggerSupplier.get() * 2);
    } else {

      climberSubsystem.stop();
    }
  }

  @Override
  public boolean isFinished() {

    return false;
  }

  @Override
  public void end(boolean interrupted) {}
}
