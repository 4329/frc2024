package frc.robot.commands.armCommands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.ArmAngleSubsystem;
import frc.robot.utilities.ArmAngle;

public class ArmAngleCommand extends Command {

    private ArmAngleSubsystem armAngleSubsystem;
    private ArmAngle armAngle;

    public ArmAngleCommand(ArmAngleSubsystem armAngleSubsystem, ArmAngle armAngle) {
        this.armAngleSubsystem = armAngleSubsystem;
        this.armAngle = armAngle;
        addRequirements(armAngleSubsystem);

    }

    @Override
    public void initialize() {

        armAngleSubsystem.setArmAngle(armAngle);

    }

    @Override
    public boolean isFinished() {
        return armAngleSubsystem.atSetpoint();
    }
}
