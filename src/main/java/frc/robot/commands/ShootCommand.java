package frc.robot.commands;

import edu.wpi.first.wpilibj.XboxController.Button;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.ShootSubsystem;

public class ShootCommand extends Command{
    
    private ShootSubsystem shootSubsystem;
    private double setPoint = 4500; 
    

    public ShootCommand(ShootSubsystem shootSubsystem){
        this.shootSubsystem = shootSubsystem;
        addRequirements(shootSubsystem);
    }

    @Override
    public void initialize() {
        shootSubsystem.changeSetpoint(setPoint);
    }

    @Override
    public void execute() {

        
    }


    @Override
    public boolean isFinished() {

        return false;
    }

    @Override
    public void end(boolean interrupted) {
        shootSubsystem.changeSetpoint(0);
        shootSubsystem.stop();
    }


}
