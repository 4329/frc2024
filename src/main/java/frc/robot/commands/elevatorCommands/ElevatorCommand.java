package frc.robot.commands.elevatorCommands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.ElevatorSubsystem;
import frc.robot.utilities.ElevatorSetpoints;

public class ElevatorCommand extends Command {

    private ElevatorSubsystem elevatorSubsystem;
    private ElevatorSetpoints elevatorSetpoints;

    public ElevatorCommand(ElevatorSubsystem elevatorSubsystem, ElevatorSetpoints elevatorSetpoints) {

        this.elevatorSubsystem = elevatorSubsystem;
        this.elevatorSetpoints = elevatorSetpoints;
        addRequirements(elevatorSubsystem);

    }

    @Override
    public void initialize() {
        
        elevatorSubsystem.setElevatorPose(elevatorSetpoints);

    }


    @Override
    public boolean isFinished() {
        return elevatorSubsystem.atSetpoint();
    }
}
