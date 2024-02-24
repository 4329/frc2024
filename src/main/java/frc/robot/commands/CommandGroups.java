package frc.robot.commands;

import org.junit.runners.ParentRunner;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import frc.robot.commands.armCommands.ArmAngleCommand;
import frc.robot.commands.driveCommands.CenterOnTargetCommand;
import frc.robot.commands.indexCommands.IndexCommand;
import frc.robot.commands.indexCommands.IndexReverseForShotCommand;
import frc.robot.commands.indexCommands.IndexSensorCommand;
import frc.robot.commands.indexCommands.OutdexCommand;
import frc.robot.commands.intakeOuttakeCommands.IntakeCommand;
import frc.robot.commands.intakeOuttakeCommands.IntakeSensorCommand;
import frc.robot.commands.intakeOuttakeCommands.OutakeCommand;
import frc.robot.commands.shootCommands.ShootCommand;
import frc.robot.commands.shootCommands.ShooterAimCommand;
import frc.robot.subsystems.ArmAngleSubsystem;
import frc.robot.subsystems.IndexSubsystem;
import frc.robot.subsystems.IntakeSubsystem;
import frc.robot.subsystems.LimlihSubsystem;
import frc.robot.subsystems.LineBreakSensorSubsystem;
import frc.robot.subsystems.ShootSubsystem;
import frc.robot.subsystems.VisionSubsystem;
import frc.robot.subsystems.swerve.Drivetrain;
import frc.robot.utilities.ArmAngle;

public class CommandGroups {

        public static Command intakeFull(IntakeSubsystem intakeSubsystem, IndexSubsystem indexSubsystem) {

                return new ParallelCommandGroup(

                                new IntakeCommand(intakeSubsystem),
                                new IndexCommand(indexSubsystem)

                );

        }

        public static Command intakeWithLineBreakSensor(IntakeSubsystem intakeSubsystem, IndexSubsystem indexSubsystem,
                        LineBreakSensorSubsystem lineBreakSensorSubsystem) {

                return new SequentialCommandGroup(
                                new ParallelCommandGroup(
                                                new IntakeSensorCommand(intakeSubsystem, lineBreakSensorSubsystem),
                                                new IndexSensorCommand(lineBreakSensorSubsystem, indexSubsystem)

                                ),
                                new IndexReverseForShotCommand(lineBreakSensorSubsystem, indexSubsystem));

        }

        public static Command outakeFull(IntakeSubsystem intakeSubsystem, IndexSubsystem indexSubsystem) {

                return new ParallelCommandGroup(

                                new OutakeCommand(intakeSubsystem),
                                new OutdexCommand(indexSubsystem)

                );

        }

        public static Command releaseToShoot(ShootSubsystem shootSubsystem, IndexSubsystem indexSubsystem) {

                return new ParallelCommandGroup(
                                new IndexCommand(indexSubsystem).withTimeout(3),
                                new ShootCommand(shootSubsystem).withTimeout(3));

        }

        public static Command aimAndShoot(ShootSubsystem shootSubsystem, Drivetrain m_robotDrive,
                        IndexSubsystem indexSubsystem, LimlihSubsystem limlihSubsystem,
                        CommandXboxController driverController,
                        ArmAngleSubsystem armAngleSubsystem) {

                return new SequentialCommandGroup(

                                new ParallelCommandGroup(
                                                new CenterOnTargetCommand(limlihSubsystem, m_robotDrive, 4,
                                                                driverController).withTimeout(1.5),
                                                new ShooterAimCommand(limlihSubsystem, armAngleSubsystem)
                                                                .withTimeout(1.5)

                                ),

                                new ParallelCommandGroup(

                                                new ShootCommand(shootSubsystem),

                                                new SequentialCommandGroup(

                                                                new WaitCommand(2),

                                                                new IndexCommand(indexSubsystem)

                                                )

                                ));

        }

        public static Command holdShot(ShootSubsystem shootSubsystem, Drivetrain drivetrain,
                        VisionSubsystem visionSubsystem, CommandXboxController driverController,
                        ArmAngleSubsystem armAngleSubsystem) {

                return new ParallelCommandGroup(
                                new CenterOnTargetCommand(visionSubsystem, drivetrain, 0, driverController),
                                new ShooterAimCommand(visionSubsystem, armAngleSubsystem),
                                new ShootFireCommand(shootSubsystem));

        };
}