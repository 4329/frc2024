package frc.robot.commands.indexCommands;

import java.util.Map;

import edu.wpi.first.networktables.GenericEntry;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.commands.armCommands.ArmAngleCommand;
import frc.robot.commands.shootCommands.ShuffleBoardShootCommand;
import frc.robot.subsystems.ArmAngleSubsystem;
import frc.robot.subsystems.IndexSubsystem;
import frc.robot.subsystems.IntakeSubsystem;
import frc.robot.subsystems.LightSubsystem;
import frc.robot.subsystems.LightSubsystem.LEDPattern;
import frc.robot.subsystems.LineBreakSensorSubsystem;
import frc.robot.utilities.ArmAngle;

public class AmpOutdexSensorCommand extends Command {

  private LineBreakSensorSubsystem lineBreakSensorSubsystem;
  private IndexSubsystem indexSubsystem;
  private ArmAngleSubsystem armAngleSubsystem;
  private IntakeSubsystem intakeSubsystem;
  private LightSubsystem lightSubsystem;
  private int checks = 0;
  private GenericEntry amping;

  public AmpOutdexSensorCommand(
      LineBreakSensorSubsystem lineBreakSensorSubsystem,
      IndexSubsystem indexSubsystem,
      ArmAngleSubsystem armAngleSubsystem,
      IntakeSubsystem intakeSubsystem,
      LightSubsystem lightSubsystem) {
    this.lineBreakSensorSubsystem = lineBreakSensorSubsystem;
    this.indexSubsystem = indexSubsystem;
    this.armAngleSubsystem = armAngleSubsystem;
    this.intakeSubsystem = intakeSubsystem;
    this.lightSubsystem = lightSubsystem;

    amping = Shuffleboard.getTab("RobotData").add("Amping", false).withPosition(7, 4).withSize(3, 2).withProperties(Map.of("Color when true", "FF0000", "Color when false", "000000")).getEntry();

    addRequirements(lineBreakSensorSubsystem, indexSubsystem, armAngleSubsystem, intakeSubsystem);
  }

  @Override
  public void initialize() {
    checks = 0;
    new ArmAngleCommand(armAngleSubsystem, ArmAngle.AMPDEX);

    LineBreakSensorSubsystem.NoteStore.setNoted(false);
    lightSubsystem.setLEDPattern(LEDPattern.RED);
    amping.setBoolean(true);
  }

  @Override
  public void execute() {
    if (armAngleSubsystem.atSetpoint()) {
      indexSubsystem.backInFrontOut();
      intakeSubsystem.out();
      if (lineBreakSensorSubsystem.isNotBroken()) checks++;
    }
  }

  @Override
  public boolean isFinished() {
    return checks >= 75 && lineBreakSensorSubsystem.isNotBroken();
  }

  @Override
  public void end(boolean interrupted) {
    lightSubsystem.setLEDPattern(LEDPattern.NOTHING);
    amping.setBoolean(false);

    armAngleSubsystem.setArmAngle(ArmAngle.INTAKE);
    indexSubsystem.stop();
    intakeSubsystem.stop();
  }
}
