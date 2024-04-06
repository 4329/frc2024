package frc.robot.utilities;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.lightSubsystem.LightSubsystem;

public class LightCommandScheduler {
  private Command currentCommand;
  private Command defaultCommand;

  public LightCommandScheduler(LightSubsystem lightSubsystem) {
    defaultCommand = new Command() {};

    new Thread(
            () -> {
              while (true) {
                if (currentCommand != null) {
                  currentCommand.execute();
                  if (currentCommand.isFinished()) {
                    currentCommand.end(false);
                    currentCommand = null;

                    if (defaultCommand != null) defaultCommand.initialize();
                  }
                } else if (defaultCommand != null) {
                  defaultCommand.execute();
                }

                lightSubsystem.noIWantMyPeriodic();
              }
            },
            "Light Command Thread")
        .start();
  }

  public void scheduleCommand(Command command) {
    command.initialize();
    if (currentCommand != null) currentCommand.end(true);
    else if (defaultCommand != null) defaultCommand.end(true);
    currentCommand = command;
  }

  public void cancelCommand(Command command) {
    if (currentCommand == command) {
      currentCommand.end(true);
      currentCommand = null;
    } else {
      System.out.println("That wasn't even scheduled in the first place, idiot");
    }
  }

  public void setDefaultCommand(Command command) {
    command.initialize();
    defaultCommand = command;
  }

  public boolean isScheduled(Command command) {
    return currentCommand != null ? currentCommand == command : defaultCommand == command;
  }
}
