package frc.robot.utilities;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.lightSubsystem.LightSubsystem;

public class LightCommand extends Command {
  private LightCommandScheduler lightCommandScheduler;

  public LightCommand(LightSubsystem lightSubsystem) {
    this.lightCommandScheduler = lightSubsystem.lightCommandScheduler;
  }

  @Override
  public void initialize() {
    if (MathUtils.getCallerClassName() != LightCommandScheduler.class.getName()) {
      schedule();
      return;
    }
  }

  @Override
  public void execute() {
    if (MathUtils.getCallerClassName() != LightCommandScheduler.class.getName()) {
      return;
    }
  }

  @Override
  public void schedule() {
    lightCommandScheduler.scheduleCommand(this);
  }

  @Override
  public void cancel() {
    lightCommandScheduler.cancelCommand(this);
  }

  @Override
  public boolean isScheduled() {
    return lightCommandScheduler.isScheduled(this);
  }
}
