package frc.robot.subsystems;

import edu.wpi.first.math.estimator.SwerveDrivePoseEstimator;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.subsystems.swerve.Drivetrain;

public class PoseEstimationSubsystem extends SubsystemBase {
    public class stupid {
        public int a;
    }

    private final Drivetrain drivetrain;
    private final LimlihSubsystem limlihSubsystem;
    private final SwerveDrivePoseEstimator estimator;

    public PoseEstimationSubsystem(Drivetrain drivetrain, LimlihSubsystem limlihSubsystem) {
        this.drivetrain = drivetrain;
        this.limlihSubsystem = limlihSubsystem;

        estimator = new SwerveDrivePoseEstimator(
            Constants.DriveConstants.kDriveKinematics,
            drivetrain.getGyro(),
            drivetrain.getModulePositions(),
            getPose()
        );
    }

    public Pose2d getPose() {
        return estimator.getEstimatedPosition();
    }

    @Override
    public void periodic() {
        estimator.update(drivetrain.getGyro(), drivetrain.getModulePositions());
    }
}
