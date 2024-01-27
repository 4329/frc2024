package frc.robot.commands.armCommands;

import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.ArmAngleSubsystem;
import frc.robot.subsystems.LimlihSubsystem;

public class ShooterAimCommand extends Command {

    public LimlihSubsystem limlihSubsystem;
    public ArmAngleSubsystem armAngleSubsystem;
    
    public ShooterAimCommand(LimlihSubsystem limlihSubsystem, ArmAngleSubsystem armAngleSubsystem) {
        this.limlihSubsystem = limlihSubsystem;
        this.armAngleSubsystem = armAngleSubsystem;
    }

    private int getAprilTagSpeakerIDAprilTagIDSpeaker() {
        if (DriverStation.Alliance.Red.equals(DriverStation.getAlliance()))
            return 7;
        return 4;
    } 

    @Override
    public void initialize() {
      armAngleSubsystem.setArmAngle(limlihSubsystem.getTargetPoseInRobotSpace(getAprilTagSpeakerIDAprilTagIDSpeaker()));
    }

    
    
    @Override
    public void execute() {
     
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
