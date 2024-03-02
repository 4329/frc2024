package frc.robot.commands;

import org.photonvision.estimation.VisionEstimation;

import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.IndexSubsystem;
import frc.robot.subsystems.ShootSubsystem;
import frc.robot.subsystems.VisionSubsystem;
import frc.robot.utilities.AprilTagUtil;

public class ShooterShotCommand extends Command {

    private ShootSubsystem shootSubsystem;
    private IndexSubsystem indexSubsystem;
    private VisionSubsystem visionSubsystem;
   private double setpoint = 2500;
    private Timer timer = new Timer();

    public ShooterShotCommand(ShootSubsystem shootSubsystem, IndexSubsystem indexSubsystem, VisionSubsystem visionSubsystem) {
        this.shootSubsystem = shootSubsystem;
        this.indexSubsystem = indexSubsystem;
        this.visionSubsystem = visionSubsystem;
        addRequirements(shootSubsystem);

    }

    @Override
    public void initialize() {
        
        timer.reset();
        
        if (visionSubsystem.getTargetVisible(AprilTagUtil.getAprilTagSpeakerIDAprilTagIDSpeaker())) {
            Pose3d pose3d = visionSubsystem.getTargetPoseInRobotSpace(AprilTagUtil.getAprilTagSpeakerIDAprilTagIDSpeaker());
            if (pose3d != null) {
                
                shootSubsystem.shooterDistance(pose3d);
            }

        }
    }

    @Override
    public void execute() {

        if (shootSubsystem.aboveSetpoint() == true) {

            timer.start();
            indexSubsystem.inForShot();


        }

    }

    @Override
    public boolean isFinished() {


        return timer.hasElapsed(0.4);
    }

    @Override
    public void end(boolean interrupted) {

        shootSubsystem.stop();
        indexSubsystem.stop();
        timer.stop();

    }

}
