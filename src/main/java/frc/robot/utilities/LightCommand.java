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
    lightCommandScheduler.scheduleCommand(this);
  }

  public void lightInitialize() {}

  public void lightExecute() {}

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

  @Override
  public void end(boolean interrupted) {
    lightCommandScheduler.cancelCommand(this);
  }

  @Override
  public boolean runsWhenDisabled() {
    return true;
  }
}
