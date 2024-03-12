package frc.robot.commands.shootCommands;

import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.VisionSubsystem;
import frc.robot.subsystems.shoot.ShootSubsystem;
import frc.robot.subsystems.shoot.ShootSubsystem;
import frc.robot.utilities.AprilTagUtil;

public class ShotRevCommand extends Command {

    private ShootSubsystem shootSubsystem;
    private VisionSubsystem visionSubsystem;
    private int checks = 0;
    private double setpoint = 2800;

    public ShotRevCommand(ShootSubsystem shootSubsystem, VisionSubsystem visionSubsystem) {
        this.shootSubsystem = shootSubsystem;
        this.visionSubsystem = visionSubsystem;
        addRequirements(shootSubsystem);

    }

    @Override
    public void initialize() {

        checks = 0;
        
        if (visionSubsystem.getTargetVisible(AprilTagUtil.getAprilTagSpeakerIDAprilTagIDSpeaker())) {
            Pose3d pose3d = visionSubsystem.getTargetPoseInRobotSpace(AprilTagUtil.getAprilTagSpeakerIDAprilTagIDSpeaker());
            if (pose3d != null) {

                shootSubsystem.shooterDistance(pose3d);
            }

        } else{
        shootSubsystem.changeSetpoint(setpoint);
    }
}

    @Override
    public void execute() {

        System.out.println("there are " + checks + " many checks");
        if (shootSubsystem.aboveSetpoint()) {

            checks++;

        }
    }

    @Override
    public boolean isFinished() {

        System.out.println("wafsanlklkjealkjflkdsajfkdsa   " + checks);
        if (checks >= 3) {
            return true;
        }
        return false;

    }

}
