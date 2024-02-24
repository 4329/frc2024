package frc.robot.commands.driveCommands;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.networktables.GenericEntry;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.subsystems.VisionSubsystem;
import frc.robot.subsystems.VisionSubsystem;
import frc.robot.subsystems.swerve.Drivetrain;

public class DriveToTargetCommand extends Command {

    private final Drivetrain drivetrain;
    private final VisionSubsystem visionSubsystem;

    private final PIDController rotationPID;
    private final PIDController forwardsbackwardsPidController;
    private final PIDController leftrightPidController;

    private final int targetId;
    private final double targetDistance;

    public DriveToTargetCommand(Drivetrain drivetrain, VisionSubsystem visionSubsystem, int targetId,
            double targetDistance) {
        this.drivetrain = drivetrain;
        this.visionSubsystem = visionSubsystem;
        
        rotationPID = new PIDController(0.035, 0, 0);
        rotationPID.setTolerance(0.2);
        rotationPID.setSetpoint(0);

        this.forwardsbackwardsPidController = new PIDController(0.7 / 2, 0, 0.);// (0.7, 0, 0)
        forwardsbackwardsPidController.setTolerance(0.5);

        this.leftrightPidController = new PIDController(0.58 / 2, 0, 0);// (0.58, 0, 0.0001)
        leftrightPidController.setTolerance(0.38);

        this.targetId = targetId;
        this.targetDistance = targetDistance;

        addRequirements(drivetrain, (SubsystemBase) visionSubsystem);
    }

    @Override
    public void initialize() {
        rotationPID.setSetpoint(0);
        forwardsbackwardsPidController.setSetpoint(targetDistance);
        leftrightPidController.setSetpoint(0);
    }

    @Override
    public void execute() {
        if (visionSubsystem.CameraConnected() && visionSubsystem.getTargetVisible(targetId)) {
            double rotOutput = rotationPID.calculate(visionSubsystem.getTargetX(targetId));
            double forwardsbackwardsOutput = forwardsbackwardsPidController
                    .calculate(visionSubsystem.getTargetSpacePose(targetId).getZ());
            double leftrightOutput = leftrightPidController
                    .calculate(-visionSubsystem.getTargetSpacePose(targetId).getX());
            drivetrain.drive(forwardsbackwardsOutput, leftrightOutput, rotOutput, true);
        } else {
            drivetrain.drive(0, 0, 0, false);
        }
    }

    @Override
    public boolean isFinished() {
        return rotationPID.atSetpoint() &&
                forwardsbackwardsPidController.atSetpoint() &&
                leftrightPidController.atSetpoint();

    }

    @Override
    public void end(boolean interrupted) {
        drivetrain.stop();
    }

}