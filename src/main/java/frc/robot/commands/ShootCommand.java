package frc.robot.commands;

import edu.wpi.first.wpilibj.XboxController.Button;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.ShootSubsystem;

public class ShootCommand extends Command{
    
    private ShootSubsystem shootSubsystem;
    public Button m_Y; 
    

    public ShootCommand(ShootSubsystem shootSubsystem){
        this.shootSubsystem = shootSubsystem;
        addRequirements(shootSubsystem);
    }

    @Override
    public void initialize() {
        
    }

    @Override
    public void execute() {
        System.out.println(shootSubsystem.getVelocityRPM());
        shootSubsystem.changeSetpoint();
        //shootSubsystem.shoot();
    }


    @Override
    public boolean isFinished() {

        return false;
    }

    @Override
    public void end(boolean interrupted) {
        shootSubsystem.stop();
    }


}
