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

  private final double tolerance = 0.1;
  private double setpoint = 0;

  double ticksPerRad = ArmAngle.HORIZONTAL.getValue() / ((59.5 * Math.PI) / (180.0));

  private double rateOfChange = 0.2;

  private GenericEntry setpointGE;
  private GenericEntry positionGE;
  private GenericEntry velocity;

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
    setpoint = armTable.getOutput(Math.sqrt(Math.pow(pose.getZ(), 2) + Math.pow(pose.getX(), 2)));
  }

  public void armInterpolationTable() {
    armTable =
        new LinearInterpolationTable(
            new Point2D.Double(0.0, 0),
            new Point2D.Double(1.8, 0),
            new Point2D.Double(2, 0),
            new Point2D.Double(2.2, 1),
            new Point2D.Double(2.4, 2.8), //
            // new Point2D.Double(2.6, 2),
            // new Point2D.Double(2.8, 2.78),
            new Point2D.Double(3, 4.14), //
            // new Point2D.Double(3.2, 3.6),
            // new Point2D.Double(3.4, 4),
            new Point2D.Double(3.6, 5.6), //
            // new Point2D.Double(3.8, 4.68),
            new Point2D.Double(4, 5.8), //
            // new Point2D.Double(4.2, 5.2),
            // new Point2D.Double(4.4, 5.52),
            // new Point2D.Double(4.6, 5.8),
            // new Point2D.Double(4.8, 6),
            new Point2D.Double(5, 6.4), //
            // new Point2D.Double(5.2, 6.4),
            // new Point2D.Double(5.4, 6.4),
            // new Point2D.Double(5.6, 6.55),
            new Point2D.Double(10, 6.6));
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
