package frc.robot.commands.indexCommands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.commands.armCommands.ArmAngleCommand;
import frc.robot.subsystems.ArmAngleSubsystem;
import frc.robot.subsystems.IndexSubsystem;
import frc.robot.subsystems.IntakeSubsystem;
import frc.robot.subsystems.LineBreakSensorSubsystem;
import frc.robot.utilities.ArmAngle;

public class AmpOutdexSensorCommand extends Command {

  private LineBreakSensorSubsystem lineBreakSensorSubsystem;
  private IndexSubsystem indexSubsystem;
  private ArmAngleSubsystem armAngleSubsystem;
  private IntakeSubsystem intakeSubsystem;
  private int checks = 0;
  private boolean isBroken;

  public AmpOutdexSensorCommand(
      LineBreakSensorSubsystem lineBreakSensorSubsystem,
      IndexSubsystem indexSubsystem,
      ArmAngleSubsystem armAngleSubsystem,
      IntakeSubsystem intakeSubsystem) {
    this.lineBreakSensorSubsystem = lineBreakSensorSubsystem;
    this.indexSubsystem = indexSubsystem;
    this.armAngleSubsystem = armAngleSubsystem;
    this.intakeSubsystem = intakeSubsystem;
    addRequirements(lineBreakSensorSubsystem, indexSubsystem, armAngleSubsystem, intakeSubsystem);
  }

  @Override
  public void initialize() {
    checks = 0;
    this.isBroken = !lineBreakSensorSubsystem.isNotBroken();
    new ArmAngleCommand(armAngleSubsystem, ArmAngle.AMPDEX);
  }

  @Override
  public void execute() {

    if (!lineBreakSensorSubsystem.isNotBroken() && armAngleSubsystem.atSetpoint()) {
      indexSubsystem.backInFrontOut();
      intakeSubsystem.out();

    } else if (lineBreakSensorSubsystem.isNotBroken() && armAngleSubsystem.atSetpoint()) {
      indexSubsystem.backInFrontOut();
      intakeSubsystem.out();
      checks++;
    }
  }

  @Override
  public boolean isFinished() {
    if (checks >= 30 && lineBreakSensorSubsystem.isNotBroken()) {

      return true;
    } else {
      return false;
    }
  }

  @Override
  public void end(boolean interrupted) {
    armAngleSubsystem.setArmAngle(ArmAngle.INTAKE);
    indexSubsystem.stop();
    intakeSubsystem.stop();
  }
}
