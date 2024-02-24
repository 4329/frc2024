package frc.robot.subsystems;

import org.littletonrobotics.junction.Logger;

import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class LoggingSubsystem extends SubsystemBase {

    private LoggedSubsystem[] subsystems;

    public LoggingSubsystem(LoggedSubsystem... subsystems) {
        this.subsystems = subsystems;
    }

    @Override
    public void periodic() {

        for (LoggedSubsystem subsystemLog : subsystems) {
            
            Logger.processInputs(subsystemLog.getClass().getName(), subsystemLog.log());   
        }
    }
    
}
