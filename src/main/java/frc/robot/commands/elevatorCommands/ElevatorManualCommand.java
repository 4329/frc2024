package frc.robot.commands.elevatorCommands;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import frc.robot.subsystems.ElevatorSubsystem;

public class ElevatorManualCommand extends Command {

    private ElevatorSubsystem elevatorSubsystem;
    private CommandXboxController controller;

    public ElevatorManualCommand(ElevatorSubsystem elevatorSubsystem, CommandXboxController controller) {

    }

    @Override
    public void initialize() {

    }

    @Override
    public void execute() {
        if (controller.getRightTriggerAxis() > 0.1) {
            elevatorSubsystem.goUp();

        } else if (controller.getLeftTriggerAxis() < 0.1) {
            elevatorSubsystem.goDown();
        } else {

            elevatorSubsystem.stop();
        }
    }

    @Override
    public boolean isFinished() {

        return false;
    }

    @Override
    public void end(boolean interrupted) {

    }

}