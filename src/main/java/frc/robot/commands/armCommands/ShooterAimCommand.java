package frc.robot.commands.armCommands;

import edu.wpi.first.math.geometry.Pose3d;
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
        addRequirements(armAngleSubsystem);
    }

    private int getAprilTagSpeakerIDAprilTagIDSpeaker() {
        if (DriverStation.Alliance.Red.equals(DriverStation.getAlliance()))
            return 7;
        return 4;
    }

    @Override
    public void initialize() {

    
    }

    @Override
    public void execute() {

        Pose3d pose3d = limlihSubsystem.getTargetPoseInRobotSpace(getAprilTagSpeakerIDAprilTagIDSpeaker());
        if (pose3d != null) {

            armAngleSubsystem.setArmAngle(pose3d);
        }
    }

    @Override
    public boolean isFinished() {

        return armAngleSubsystem.atSetpoint();
    }
}
