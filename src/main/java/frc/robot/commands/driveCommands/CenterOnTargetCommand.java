package frc.robot.commands.driveCommands;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.networktables.GenericEntry;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import frc.robot.Constants;
import frc.robot.subsystems.VisionSubsystem;
import frc.robot.subsystems.VisionSubsystem;
import frc.robot.subsystems.swerve.Drivetrain;
import frc.robot.utilities.MathUtils;

public class CenterOnTargetCommand extends Command {

    private final VisionSubsystem visionSubsystem;
    private final Drivetrain drivetrain;
    private final int targetId;

    private final PIDController rotationPID;
    private final CommandXboxController xboxController;

    public CenterOnTargetCommand(VisionSubsystem visionSubsystem, Drivetrain m_drivetrain, int targetId,
            CommandXboxController xboxController) {
        this.visionSubsystem = visionSubsystem;
        this.drivetrain = m_drivetrain;
        this.targetId = targetId;
        this.xboxController = xboxController;

        rotationPID = new PIDController(0.75, 0, 0);
        rotationPID.setTolerance(0.5);
        rotationPID.setSetpoint(0);

        rotationPID.enableContinuousInput(Math.PI, 2 * Math.PI);
        
        addRequirements(visionSubsystem, m_drivetrain);
    }

    @Override
    public void initialize() {
    
    }

    
    @Override
    public void execute() {
        double rotationCalc = 0;
        if (visionSubsystem.CameraConnected() && visionSubsystem.getTargetVisible(targetId)) {
            double currentRot = -visionSubsystem.getTargetSpacePose(targetId).getRotation().toRotation2d().getRadians();
            double hopefulRot = Math.atan2(visionSubsystem.getTargetSpacePose(targetId).getX(), visionSubsystem.getTargetSpacePose(targetId).getY());
            rotationCalc = rotationPID.calculate(hopefulRot - currentRot);
            if (rotationCalc > Constants.DriveConstants.kMaxAngularSpeed)
                rotationCalc = Constants.DriveConstants.kMaxAngularSpeed;
            else if (rotationCalc < -Constants.DriveConstants.kMaxAngularSpeed)
                rotationCalc = -Constants.DriveConstants.kMaxAngularSpeed;
            else if (rotationPID.atSetpoint())
                rotationCalc = 0;

            double adjTranslation = ((Constants.DriveConstants.kMaxAngularSpeed - Math.abs(rotationCalc))
                    / Constants.DriveConstants.kMaxAngularSpeed) * 0.5;

            drivetrain.drive(
                    -inputTransform(xboxController.getLeftY())
                            * (Constants.DriveConstants.kMaxSpeedMetersPerSecond * adjTranslation),
                    -inputTransform(xboxController.getLeftX())
                            * (Constants.DriveConstants.kMaxSpeedMetersPerSecond * adjTranslation),
                    rotationCalc,
                    true);
        }
    }

    @Override
    public boolean isFinished() {
        return rotationPID.atSetpoint();
    }

    private double inputTransform(double input) {
        return MathUtils.singedSquare(MathUtils.applyDeadband(input));
    }

    @Override
    public void end(boolean interrupted) {
        drivetrain.stop();
    }

}
