package frc.robot.commands.shootCommands;

import edu.wpi.first.wpilibj.XboxController.Button;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.shoot.ShootSubsystem;
import frc.robot.subsystems.shoot.ShootSubsystem;

public class ShootFireCommand extends Command{
    
    private ShootSubsystem shootSubsystem;
    private double setPoint = 5000; 
    

    public ShootFireCommand(ShootSubsystem shootSubsystem){
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


}
