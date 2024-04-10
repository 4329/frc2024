package frc.robot.subsystems;

import com.revrobotics.CANSparkBase.IdleMode;
import com.revrobotics.CANSparkBase.SoftLimitDirection;
import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.SparkLimitSwitch.Type;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.trajectory.TrapezoidProfile.Constraints;
import edu.wpi.first.math.trajectory.TrapezoidProfile.State;
import edu.wpi.first.networktables.GenericEntry;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj2.command.ProfiledPIDSubsystem;
import frc.robot.Constants;
import frc.robot.Model.ArmAngleLogAutoLogged;
import frc.robot.subsystems.LoggingSubsystem.LoggedSubsystem;
import frc.robot.utilities.ArmAngle;
import frc.robot.utilities.LinearInterpolationTable;
import frc.robot.utilities.SparkFactory;
import java.awt.geom.Point2D;
import org.littletonrobotics.junction.inputs.LoggableInputs;

public class ArmAngleSubsystem extends ProfiledPIDSubsystem implements LoggedSubsystem {

  private CANSparkMax armMotor;

  private RelativeEncoder armEncoder;
  // private SparkPIDController armPID;

  private boolean brake;

  private final double tolerance = 0.1;
  private double setpoint = 0;

  double ticksPerRad = ArmAngle.HORIZONTAL.getValue() / ((59.5 * Math.PI) / (180.0));

  private final double speakerHeight = 2.15;
  private double speakerMod = 0;
  private double rateOfChange = 0.2;
  private final double goalConstant = speakerHeight - Constants.LimlihConstants.limlihHeight;
  private GenericEntry setpointGE;
  private GenericEntry positionGE;
  private GenericEntry velocity;
  private GenericEntry radians2RotatedGE;
  private GenericEntry speakerModGE;
  private GenericEntry armpidthing;
  private GenericEntry armpidthingagain;
  private ArmAngleLogAutoLogged armAngleLogAutoLogged;
  private LinearInterpolationTable armTable;

  public ArmAngleSubsystem() {
    super(new ProfiledPIDController(3, 0, 0.005, new Constraints(100, 75)));

    armInterpolationTable();
    armAngleLogAutoLogged = new ArmAngleLogAutoLogged();
    armMotor = SparkFactory.createCANSparkMax(Constants.CANIDConstants.armRotation1, false);
    armEncoder = armMotor.getEncoder();

    armMotor.enableSoftLimit(SoftLimitDirection.kForward, true);
    armMotor.enableSoftLimit(SoftLimitDirection.kReverse, true);
    armMotor.setIdleMode(IdleMode.kBrake);
    armMotor.setSoftLimit(SoftLimitDirection.kForward, (float) ArmAngle.FULL.getValue());
    armMotor.setSoftLimit(SoftLimitDirection.kReverse, (float) ArmAngle.ZERO.getValue());
    armMotor.enableVoltageCompensation(Constants.voltageCompensation);

    armEncoder.setPosition(0);

    armEncoder.setPositionConversionFactor(1 / Constants.ArmAngleSubsystemConstants.armGearRatio);

    setpointGE = Shuffleboard.getTab("Arm Angle").add("arm setpoint", 0).getEntry();
    positionGE = Shuffleboard.getTab("Arm Angle").add("arm position", 0).getEntry();
    velocity = Shuffleboard.getTab("Arm Angle").add("velocity", 0).getEntry();
    armpidthing = Shuffleboard.getTab("Arm Angle").add("pidthing POSITION", 0).getEntry();
    armpidthingagain = Shuffleboard.getTab("Arm Angle").add("pidthing VELOCITY", 0).getEntry();
    // speakerModGE = Shuffleboard.getTab("Arm Angle").add("speakerMod", 0).getEntry();
    // radians2RotatedGE = Shuffleboard.getTab("Arm Angle").add("Radians2Rotated", 0).getEntry();

    armMotor.burnFlash();
    enable();
  }

  public void incrementSetpoint(double increment) {
    setpoint += increment;
  }

  public boolean endSensor() {

    return armMotor.getReverseLimitSwitch(Type.kNormallyOpen).isPressed();
  }

  public void resetZero() {

    setpoint = 0;
    armEncoder.setPosition(0);
  }

  public void setArmAngle(Pose3d pose) {

    // double radians1 = Math.atan2(goalConstant, pose.getZ());
    // speakerMod = speakerHeight - (radians1 * 0.6) ;
    // radians1 = MathUtils.clamp(0, 1.03, radians1); //was 1.22

    // double radians2 = Math.atan2(speakerMod, pose.getZ());

    // speakerModGE.setDouble(speakerMod);
    // radiansRotatedGE.setDouble(radians1);
    // radians2RotatedGE.setDouble(radians2);

    // setpoint = ArmAngle.HORIZONTAL.getValue() - (radians2 * ticksPerRad);

    setpoint = armTable.getOutput(Math.sqrt(Math.pow(pose.getZ(), 2) + Math.pow(pose.getX(), 2)));
  }

  public void armInterpolationTable() {

    armTable =
        new LinearInterpolationTable(
            new Point2D.Double(0.0, 1),
            new Point2D.Double(1.7, 1),
            new Point2D.Double(1.9, 1.1),
            new Point2D.Double(2.1, 1.75),
            new Point2D.Double(2.3, 2.55),
            new Point2D.Double(2.5, 3.5),
            new Point2D.Double(2.7, 4.05),
            new Point2D.Double(2.9, 4.52),
            new Point2D.Double(3.1, 4.9),
            new Point2D.Double(3.3, 5.15),
            new Point2D.Double(3.5, 5.7),
            new Point2D.Double(3.7, 5.78),
            new Point2D.Double(3.9, 5.92),
            new Point2D.Double(4.1, 6.32),
            new Point2D.Double(4.3, 6.37),
            new Point2D.Double(4.5, 6.48),
            new Point2D.Double(4.7, 6.85),
            new Point2D.Double(4.9, 6.9),
            // new Point2D.Double(5.1, 6.78),
            // new Point2D.Double(5.3, 6.82),
            new Point2D.Double(5.5, 6.73),
            new Point2D.Double(10, 6.73));
    // new Point2D.Double(2.87, 1.85));
    // new Point2D.Double(3, 1.59));
    // new Point2D.Double(3.2, 1.48),
    // new Point2D.Double(4, 1.5));

  }

  public boolean atSetpoint() {
    return Math.abs(armEncoder.getPosition() - setpoint) <= tolerance;
  }

  @Override
  public LoggableInputs log() {
    armAngleLogAutoLogged.setpoint = setpoint;
    armAngleLogAutoLogged.position = armEncoder.getPosition();
    armAngleLogAutoLogged.radians = armEncoder.getPosition() / 4.1284; // subject to change
    return armAngleLogAutoLogged;
  }

  public void armPositonUp() {
    if (setpoint < ArmAngle.SHOOTERARMAMP.getValue() - rateOfChange) {
      setpoint = Math.min(setpoint + rateOfChange, ArmAngle.SHOOTERARMAMP.getValue());
    } else {
      setpoint = ArmAngle.FULL.getValue();
    }
  }

  public void armPositonDown() {
    if (setpoint > ArmAngle.ZERO.getValue() + rateOfChange) {
      setpoint = Math.max(setpoint - rateOfChange, ArmAngle.ZERO.getValue());
    } else {
      setpoint = ArmAngle.ZERO.getValue();
    }
  }

  public void changeArmPosition(double moveAmount) {
    setpoint =
        Math.min(
            Math.max(setpoint + moveAmount, ArmAngle.ZERO.getValue()),
            ArmAngle.SHOOTERARMAMP.getValue());
  }

  @Override
  public void periodic() {
    setpointGE.setDouble(setpoint);
    positionGE.setDouble(armEncoder.getPosition());
    velocity.setDouble(armEncoder.getVelocity());
    armpidthing.setDouble(getController().getPositionError());
    armpidthingagain.setDouble(getController().getVelocityError());
    setGoal(setpoint);

    super.periodic();
  }

  public void setArmAngle(ArmAngle armAngle) {

    setpoint = armAngle.getValue();
  }

  public double getAngleRadians() {
    return setpoint / ticksPerRad;
  }

  @Override
  protected void useOutput(double output, State setpoint) {
    armMotor.setVoltage(output);
  }

  @Override
  protected double getMeasurement() {
    return armEncoder.getPosition();
  }
}
