package frc.robot.commands.armCommands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.ArmAngleSubsystem;

public class ArmUpCommand extends Command {
    
     public ArmAngleSubsystem armAngleSubsystem;

    public ArmUpCommand(ArmAngleSubsystem armAngleSubsystem) {
        this.armAngleSubsystem = armAngleSubsystem;
        
        addRequirements(armAngleSubsystem);

    }

    @Override
    public void initialize() {
      
        armAngleSubsystem.armPositonUp();
    }

    @Override
    public boolean isFinished() {
        return armAngleSubsystem.atSetpoint();
    }
    
    
    

}
