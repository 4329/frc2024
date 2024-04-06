package frc.robot.utilities;

import frc.robot.subsystems.lightSubsystem.LightSubsystem;
import org.littletonrobotics.junction.Logger;

public class LightCommandScheduler implements Runnable {
  private LightCommand currentLightCommand;
  private LightCommand defaultLightCommand;
  private LightSubsystem lightSubsystem;

  public LightCommandScheduler(LightSubsystem lightSubsystem) {
    this.lightSubsystem = lightSubsystem;
  }

  @Override
  public void run() {
    while (true) {
      if (currentLightCommand != null) {
        currentLightCommand.lightExecute();

        if (currentLightCommand.isFinished()) {
          currentLightCommand.end(false);
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
    String a = currentLightCommand != null ? currentLightCommand.getName() : "";
    String b = defaultLightCommand != null ? defaultLightCommand.getName() : "";
    Logger.recordOutput("currentCommand", a);
    Logger.recordOutput("defaultCommand", b);
  }
}
