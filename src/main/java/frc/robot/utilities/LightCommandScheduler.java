package frc.robot.utilities;

import frc.robot.Model.CommandLogEntry;
import frc.robot.subsystems.lightSubsystem.LightSubsystem;
import org.littletonrobotics.junction.Logger;

public class LightCommandScheduler implements Runnable {

  private CommandLogEntry commandLogEntry;
  private String stage;

  private LightCommand currentLightCommand;
  private LightCommand defaultLightCommand;
  private LightSubsystem lightSubsystem;

  public LightCommandScheduler(LightSubsystem lightSubsystem) {
    this.lightSubsystem = lightSubsystem;
    commandLogEntry = new CommandLogEntry();
  }

  @Override
  public void run() {
    while (true) {
      if (currentLightCommand != null) {
        currentLightCommand.lightExecute();
        stage = "execute";

        if (currentLightCommand.isFinished()) {
          currentLightCommand.end(false);
          stage = "end";
          currentLightCommand = defaultLightCommand;

          if (defaultLightCommand != null) defaultLightCommand.lightInitialize();
        }
      }

      lightSubsystem.noIWantMyPeriodic();
    }
  }

  public void scheduleCommand(LightCommand command) {
    if (currentLightCommand == command) return;
    if (currentLightCommand != null) currentLightCommand.end(true);

    command.lightInitialize();
    stage = "initialize";

    currentLightCommand = command;
  }

  public void cancelCommand(LightCommand command) {
    if (currentLightCommand == command) {
      currentLightCommand = defaultLightCommand;
    } else {
      System.out.println("That wasn't even scheduled in the first place, idiot");
    }
  }

  public void setDefaultCommand(LightCommand command) {
    command.lightInitialize();
    defaultLightCommand = command;
  }

  public boolean isScheduled(LightCommand command) {
    return currentLightCommand != null && currentLightCommand == command;
  }

  public void logPlease() {
    commandLogEntry.set(currentLightCommand.getName(), stage);
    Logger.processInputs("LightCommand", commandLogEntry);
  }
}
