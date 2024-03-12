package frc.robot.commands.armCommands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.ArmAngleSubsystem;

public class MoveArmCommand extends Command {
    private final ArmAngleSubsystem armAngleSubsystem;
    private final double changeAmount;

    public MoveArmCommand(ArmAngleSubsystem armAngleSubsystem, double changeAmount) {
        this.armAngleSubsystem = armAngleSubsystem;
        this.changeAmount = changeAmount;

        addRequirements(armAngleSubsystem);
    }

    @Override
    public void execute() {
        armAngleSubsystem.changeArmPosition(changeAmount);
    }

    @Override
    public boolean isFinished() {
        return false;
    }
}
