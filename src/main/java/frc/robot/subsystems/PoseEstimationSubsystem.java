package frc.robot.subsystems;

import org.littletonrobotics.junction.AutoLog;
import org.littletonrobotics.junction.Logger;

import edu.wpi.first.math.estimator.SwerveDrivePoseEstimator;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.Watchdog;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.subsystems.swerve.Drivetrain;

public class PoseEstimationSubsystem extends SubsystemBase {

    stupideAutoLogged a;

    private final Drivetrain drivetrain;
    private final LimlihSubsystem limlihSubsystem;
    private final SwerveDrivePoseEstimator estimator;

    public PoseEstimationSubsystem(Drivetrain drivetrain, LimlihSubsystem limlihSubsystem) {
        this.drivetrain = drivetrain;
        this.limlihSubsystem = limlihSubsystem;
        a = new stupideAutoLogged();
        estimator = new SwerveDrivePoseEstimator(
                Constants.DriveConstants.kDriveKinematics,
                drivetrain.getGyro(),
                drivetrain.getModulePositions(),
                drivetrain.getPose());
    }

    public Pose2d getPose() {
        return estimator.getEstimatedPosition();
    }

    private void george() {
        if (limlihSubsystem.seeingAnything()) {
            System.out.println("iklasfdflsfa");
            estimator.addVisionMeasurement(limlihSubsystem.getPose(), Timer.getFPGATimestamp());
        }
    }

    @Override
    public void periodic() {
        estimator.update(drivetrain.getGyro(), drivetrain.getModulePositions());
        george();
        updateInputs(a);
    }

    private void updateInputs(stupide stupid) {
        stupid.a = getPose();
        Logger.processInputs("ohno", a);
    }
}
