package frc.robot.commands.wrist;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.WristSubsystem;

public class WristRotateUpCommand extends Command {
    
    WristSubsystem wristSubsystem;

    public WristRotateUpCommand(WristSubsystem wristSubsystem) {

        this.wristSubsystem = wristSubsystem;
    }
    
    @Override
    public void execute() {

        wristSubsystem.wristUp();
    }

    @Override
    public void end(boolean interrupted) {

        
    }


    
}
