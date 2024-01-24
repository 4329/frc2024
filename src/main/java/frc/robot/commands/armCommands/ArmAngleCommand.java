package frc.robot.commands.armCommands;

import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.ArmAngleSubsystem;
import frc.robot.subsystems.LimlihSubsystem;

public class ArmAngleCommand extends Command {

    public LimlihSubsystem limlihSubsystem;
    public ArmAngleSubsystem armAngleSubsystem;
    
    public ArmAngleCommand(LimlihSubsystem limlihSubsystem, ArmAngleSubsystem armAngleSubsystem) {
        this.limlihSubsystem = limlihSubsystem;
        this.armAngleSubsystem = armAngleSubsystem;
    }

    private int Alliance() {
        if (DriverStation.Alliance.Red.equals(DriverStation.getAlliance()))
            return 7;
        return 4;
    }

    @Override
    public void initialize() {
      armAngleSubsystem.setArmAngle(limlihSubsystem.getTargetPoseInRobotSpace(Alliance()));
    }
    
    @Override
    public boolean isFinished() {
       
        return armAngleSubsystem.atSetpoint();
    }
    
    @Override
    public void end(boolean interrupted) {
        // TODO Auto-generated method stub
        super.end(interrupted);
    }
    
    
}
