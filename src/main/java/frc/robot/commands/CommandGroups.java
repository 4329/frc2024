package frc.robot.commands;

import java.util.Optional;

import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import frc.robot.commands.armCommands.ArmAngleCommand;
import frc.robot.commands.armCommands.ShooterAimCommand;
import frc.robot.commands.drive.CenterOnTargetCommand;
import frc.robot.subsystems.ArmAngleSubsystem;
import frc.robot.subsystems.ElevatorSubsystem;
import frc.robot.subsystems.IndexSubsystem;
import frc.robot.subsystems.IntakeSubsystem;
import frc.robot.subsystems.LimlihSubsystem;
import frc.robot.subsystems.ShootSubsystem;
import frc.robot.subsystems.swerve.Drivetrain;
import frc.robot.utilities.ArmAngle;
import frc.robot.utilities.ElevatorSetpoints;

public class CommandGroups {

        public static Command intakeFull(IntakeSubsystem intakeSubsystem, IndexSubsystem indexSubsystem) {

                return new ParallelCommandGroup(

                                new IntakeCommand(intakeSubsystem),
                                new IndexCommand(indexSubsystem)

                );

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
                Optional<Alliance> alliance = DriverStation.getAlliance();
                int targetId = alliance.isPresent() && alliance.get().equals(DriverStation.Alliance.Red) ? 4 : 7;
                return new SequentialCommandGroup(

                                new ParallelCommandGroup(
                                                new CenterOnTargetCommand(limlihSubsystem, m_robotDrive, targetId,
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

        public static Command ampCommand(ElevatorSubsystem elevatorSubsystem,
                        ArmAngleSubsystem armAngleSubsystem,
                        ShootSubsystem shootSubsystem) {
                return new SequentialCommandGroup(
                                new ElevatorCommand(elevatorSubsystem, ElevatorSetpoints.AMP),
                                new WaitCommand(2),
                                new ArmAngleCommand(armAngleSubsystem, ArmAngle.FULL),
                                new WaitCommand(2),
                                new ShootCommand(shootSubsystem)
                                );
        }

}
