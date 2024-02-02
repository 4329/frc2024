package frc.robot.commands.drive;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.networktables.GenericEntry;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.LimlihSubsystem;
import frc.robot.subsystems.swerve.Drivetrain;

public class DriveToTargetCommand extends Command {

    private final Drivetrain drivetrain;
    private final LimlihSubsystem limlihSubsystem;

    private final PIDController rotationPID;
    private final PIDController forwardsbackwardsPidController;
    private final PIDController leftrightPidController;

    private final int targetId;
    private final double targetDistance;

    GenericEntry oaijifsd = Shuffleboard.getTab("o;adssfa").add("output", 0).getEntry();
    GenericEntry IsTargetVis = Shuffleboard.getTab("o;adssfa").add("IsVis", false).getEntry();


    public DriveToTargetCommand(Drivetrain drivetrain, LimlihSubsystem limlihSubsystem, int targetId,
            double targetDistance) {
        this.drivetrain = drivetrain;
        this.limlihSubsystem = limlihSubsystem;

        this.rotationPID = new PIDController(0.82, 0, 0);//(.82, 0, 0)
        this.forwardsbackwardsPidController = new PIDController(1, 0, 0.);//(1, 0, 0)
        this.leftrightPidController = new PIDController(0.665, 0, 0.0001);//(.665, 0, 0.0001)

        this.targetId = targetId;
        this.targetDistance = targetDistance;

        addRequirements(drivetrain, limlihSubsystem);
    }

    @Override
    public void initialize() {
        rotationPID.setSetpoint(0);
        forwardsbackwardsPidController.setSetpoint(targetDistance);
        leftrightPidController.setSetpoint(0);
    }

    @Override
    public void execute() {
        if (limlihSubsystem.getTargetVisible(targetId)) {

            double rotOutput = rotationPID.calculate(-limlihSubsystem.getCalculatedPoseRot(targetId));
            double forwardsbackwardsOutput = forwardsbackwardsPidController.calculate(limlihSubsystem.getTargetSpacePose(targetId).getZ());
            double leftrightOutput = leftrightPidController.calculate(-limlihSubsystem.getTargetSpacePose(targetId).getX());
            oaijifsd.setDouble(rotOutput);
            drivetrain.drive(forwardsbackwardsOutput, leftrightOutput, rotOutput, true);
        } else {
            drivetrain.drive(0, 0, 0, false);
        }
        IsTargetVis.setBoolean(limlihSubsystem.getTargetVisible(targetId));
    }

}
