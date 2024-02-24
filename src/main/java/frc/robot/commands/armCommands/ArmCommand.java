package frc.robot.commands.armCommands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.ArmAngleSubsystem;
import frc.robot.utilities.ArmAngle;

public class ArmCommand extends Command {

    private ArmAngleSubsystem armAngleSubsystem;
    private ArmAngle armAngle;

    public ArmCommand(ArmAngleSubsystem armAngleSubsystem, ArmAngle armAngle) {
        this.armAngleSubsystem = armAngleSubsystem;
        this.armAngle = armAngle;
        addRequirements(armAngleSubsystem);
    }

    @Override
    public void execute() {
        armAngleSubsystem.setArmAngle(armAngle);
        System.out.println("lkdsjfa;lkdsajf;lsajffdsaf");
    }

    @Override
    public boolean isFinished() {
        return armAngleSubsystem.atSetpoint();
    }

    

    
    
}
