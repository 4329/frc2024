package frc.robot.commands.intakeOuttakeCommands;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import frc.robot.commands.indexCommands.IndexReverseForShotCommand;
import frc.robot.commands.indexCommands.IndexSensorCommand;
import frc.robot.utilities.ReInitCommand;

public class ToggleIntakeCommand extends ReInitCommand {
    ParallelCommandGroup intakeSensorGroup;
    IndexReverseForShotCommand indexReverseForShotCommand;

    private boolean toggled;

    public ToggleIntakeCommand(IntakeSensorCommand intakeSensorCommand, IndexSensorCommand indexSensorCommand, IndexReverseForShotCommand indexReverseForShotCommand) {
        intakeSensorGroup = intakeSensorCommand.alongWith(indexSensorCommand);
        this.indexReverseForShotCommand = indexReverseForShotCommand;
    }
    
    @Override
    public void initialize() {
        if (!toggled)
            intakeSensorGroup.schedule();
        else
            this.cancel();
        toggled = !toggled;
    }

    @Override
    public boolean isFinished() {
        return !intakeSensorGroup.isScheduled();
    }

    @Override
    public void end(boolean interrupted) {
        intakeSensorGroup.cancel();
        if (!interrupted) {
            indexReverseForShotCommand.schedule();
        }
    }
}
