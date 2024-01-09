package frc.robot.commands.wrist;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.WristSubsystem;

public class WristRotateDownCommand extends Command {
    
    WristSubsystem wristSubsystem;

    public WristRotateDownCommand(WristSubsystem wristSubsystem) {

        this.wristSubsystem = wristSubsystem;
    }
    
    @Override
    public void execute() {

        wristSubsystem.wristDown();
    }

    @Override
    public void end(boolean interrupted) {

        
    }


    
}
