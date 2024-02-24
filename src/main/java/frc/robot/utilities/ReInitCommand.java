package frc.robot.utilities;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;

public abstract class ReInitCommand extends Command {

    @Override
    public void schedule() { 
        CommandScheduler.getInstance().schedule(this);
        initialize();
    }
}
