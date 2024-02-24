package frc.robot.commands.armCommands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.ArmAngleSubsystem;
import frc.robot.utilities.ArmAngle;

public class ArmIntakeCommand extends Command {

    private ArmAngleSubsystem armAngleSubsystem;

    public ArmIntakeCommand(ArmAngleSubsystem armAngleSubsystem) {
        this.armAngleSubsystem = armAngleSubsystem;
    }

    @Override
    public void initialize() {
       armAngleSubsystem.setArmAngle(ArmAngle.INTAKE);
    
    }

    
    
}

