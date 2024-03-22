package frc.robot.commands.shootCommands;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import frc.robot.subsystems.ArmAngleSubsystem;
import frc.robot.subsystems.ShootSubsystem;
import frc.robot.subsystems.VisionSubsystem;
import frc.robot.subsystems.swerve.Drivetrain;

public class HoldShotCommand extends ParallelCommandGroup {

    public HoldShotCommand(
            ShootSubsystem shootSubsystem,
            Drivetrain drivetrain,
            VisionSubsystem visionSubsystem,
            CommandXboxController driverController,
            ArmAngleSubsystem armAngleSubsystem) {

        super(
                new ShooterAimCommand(visionSubsystem, armAngleSubsystem),
                new ShootFireCommand(shootSubsystem));
    };
}
