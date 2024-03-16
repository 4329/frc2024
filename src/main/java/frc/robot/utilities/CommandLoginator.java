package frc.robot.utilities;

import edu.wpi.first.wpilibj2.command.CommandScheduler;

public class CommandLoginator {
  public CommandLoginator() {
    init();
  }

  private void init() {

    CommandScheduler.getInstance().onCommandInitialize(new CommandConsumingLogger("initialize()"));
    CommandScheduler.getInstance().onCommandExecute(new CommandConsumingLogger("execute()"));
    CommandScheduler.getInstance().onCommandFinish(new CommandConsumingLogger("end(true)"));
    CommandScheduler.getInstance().onCommandInterrupt(new CommandConsumingLogger("end(false)"));
  }
}
