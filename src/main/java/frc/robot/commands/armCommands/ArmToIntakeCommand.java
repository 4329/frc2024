package frc.robot.commands.armCommands;

import edu.wpi.first.wpilibj2.command.InstantCommand;
import frc.robot.subsystems.ArmAngleSubsystem;
import frc.robot.utilities.ArmAngle;

public class ArmToIntakeCommand extends InstantCommand {

    private ArmAngleSubsystem armAngleSubsystem;

    public ArmToIntakeCommand(ArmAngleSubsystem armAngleSubsystem) {
        this.armAngleSubsystem = armAngleSubsystem;
        addRequirements(armAngleSubsystem);
    }

    @Override
    public void initialize() {
       armAngleSubsystem.setArmAngle(ArmAngle.INTAKE);
    

    }

    
    
}

