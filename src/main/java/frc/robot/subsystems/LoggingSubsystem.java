package frc.robot.subsystems;

import org.littletonrobotics.junction.Logger;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class LoggingSubsystem extends SubsystemBase {

    private LoggedSubsystem[] subsystems;
    public Timer timer;

    public LoggingSubsystem(LoggedSubsystem... subsystems) {
        this.subsystems = subsystems;
        timer = new Timer();
        timer.start();
    }

    @Override
    public void periodic() {

        if (timer.hasElapsed(1)) {

            for (LoggedSubsystem subsystemLog : subsystems) {

                Logger.processInputs(subsystemLog.getClass().getName(), subsystemLog.log());
            }
            timer.restart();
        }
    }

}
