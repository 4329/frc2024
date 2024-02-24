package frc.robot.commands.driveCommands;

import edu.wpi.first.wpilibj2.command.Command;

public class ChangeFieldOrientCommand extends Command{

    DriveByController driveByController;

    public ChangeFieldOrientCommand(DriveByController driveByController) {

        this.driveByController = driveByController;
    }

    @Override
    public void initialize() {

        driveByController.changeFieldOrient();
    }

    @Override
    public boolean isFinished() {

        return true;
    }

}
