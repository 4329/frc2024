package frc.robot.commands.rotation;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.ArmRotationSubsystem;


public class ArmRotateCommand extends Command {

    private ArmRotationSubsystem armRotationSubsystem;

    public ArmRotateCommand(ArmRotationSubsystem armRotationSubsystem) {

        this.armRotationSubsystem = armRotationSubsystem;
        addRequirements(armRotationSubsystem);
    }

    @Override
    public void execute() {

        armRotationSubsystem.armRotate();
    }

    @Override
    public void end(boolean interrupted) {

        armRotationSubsystem.stop();
    }

}