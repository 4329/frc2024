package frc.robot.commands.rotation;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.ArmRotationSubsystem;


public class ArmUnrotateCommand extends Command {

    private ArmRotationSubsystem armRotationSubsystem;

    public ArmUnrotateCommand(ArmRotationSubsystem armRotationSubsystem) {

        this.armRotationSubsystem = armRotationSubsystem;
        addRequirements(armRotationSubsystem);
    }

    @Override
    public void execute() {

        armRotationSubsystem.armUnrotate();
    }

    @Override
    public void end(boolean interrupted) {

        armRotationSubsystem.stop();
    }

}