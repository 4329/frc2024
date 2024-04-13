package frc.robot.subsystems;

import com.revrobotics.CANSparkBase.ControlType;
import com.revrobotics.CANSparkBase.IdleMode;
import com.revrobotics.CANSparkBase.SoftLimitDirection;
import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.SparkPIDController;
import edu.wpi.first.networktables.GenericEntry;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.Model.ClimberLogAutoLogged;
import frc.robot.subsystems.LoggingSubsystem.LoggedSubsystem;
import frc.robot.utilities.ClimberSetpoints;
import frc.robot.utilities.SparkFactory;
import org.littletonrobotics.junction.inputs.LoggableInputs;

public class ClimberSubsystem extends SubsystemBase implements LoggedSubsystem {

  private CANSparkMax climberMotor1;
  private CANSparkMax climberMotor2;
  private RelativeEncoder climberEncoder;
  private RelativeEncoder climberEncoder2;
  private SparkPIDController climberPID;
  private SparkPIDController climberPID2;
  private GenericEntry climberPositionGenericEntry;
  private GenericEntry climberActualPositionGenericEntry;
  private ClimberSetpoints targetClimb = null;

  ClimberLogAutoLogged climberLogAutoLogged;
  private final double tolerance = 0.1;
  private double setPoint = 0;
  private double climberPositionalRateOfChange = 2;
  private boolean pitZero = false;
  private double pitRate = 0.08;

  // private SparkAnalogSensor sparkAnalogSensor;
  // GenericEntry digiput;
  // GenericEntry digiputLimit;

  // private SparkLimitSwitch m_reverseLimit;

  public ClimberSubsystem() {
    climberLogAutoLogged = new ClimberLogAutoLogged();

    climberMotor1 = SparkFactory.createCANSparkMax(Constants.CANIDConstants.climberMotor1, true);
    climberMotor2 = SparkFactory.createCANSparkMax(Constants.CANIDConstants.climberMotor2, false);
    climberPID = climberMotor1.getPIDController();
    climberPID2 = climberMotor2.getPIDController();

    // climberPID.setSmartMotionMinOutputVelocity(1, 0);
    climberEncoder = climberMotor1.getEncoder();
    climberEncoder2 = climberMotor2.getEncoder();
    climberMotor1.enableSoftLimit(SoftLimitDirection.kForward, true);
    climberMotor1.enableSoftLimit(SoftLimitDirection.kReverse, true);
    climberMotor2.enableSoftLimit(SoftLimitDirection.kForward, true);
    climberMotor2.enableSoftLimit(SoftLimitDirection.kReverse, true);
    climberMotor1.setIdleMode(IdleMode.kBrake);
    climberMotor2.setIdleMode(IdleMode.kBrake);
    climberMotor1.setSoftLimit(SoftLimitDirection.kForward, ClimberSetpoints.LEFTMAX.getValue());
    climberMotor2.setSoftLimit(SoftLimitDirection.kForward, ClimberSetpoints.RIGHTMAX.getValue());
    climberMotor1.setSoftLimit(SoftLimitDirection.kReverse, ClimberSetpoints.ZERO.getValue());
    climberMotor2.setSoftLimit(SoftLimitDirection.kReverse, ClimberSetpoints.ZERO.getValue());
    climberMotor1.enableVoltageCompensation(Constants.voltageCompensation);
    climberMotor2.enableVoltageCompensation(Constants.voltageCompensation);
    climberMotor1.setSmartCurrentLimit(40);
    climberMotor2.setSmartCurrentLimit(40);
    climberPositionGenericEntry =
        Shuffleboard.getTab("Arm Angle").add("Climber desired pos", 0).getEntry();
    climberActualPositionGenericEntry =
        Shuffleboard.getTab("Arm Angle").add("Climber Actual pos", 0).getEntry();

    climberEncoder.setPosition(0);
    climberEncoder2.setPosition(0);

    climberPID.setP(0.5);
    climberPID.setI(0);
    climberPID.setD(0);
    climberPID.setFF(0);
    climberPID.setOutputRange(-1, 1);

    climberPID2.setP(0.5);
    climberPID2.setI(0);
    climberPID2.setD(0);
    climberPID2.setFF(0);
    climberPID2.setOutputRange(-1, 1);

    climberMotor1.burnFlash();
    climberMotor2.burnFlash();

    /*
     * digitalInput = new DigitalInput(0);
     * digiput = Shuffleboard
     * .getTab("MagenetSensor")
     * .add("MagnetSensor", 1)
     * .withWidget(BuiltInWidgets.kGraph)
     * .getEntry();
     */

    // m_reverseLimit =
    // elevatorMotor1.getReverseLimitSwitch(SparkLimitSwitch.Type.kNormallyClosed);
    // m_reverseLimit.enableLimitSwitch(false);
    // digiputLimit = Shuffleboard.getTab("MagnetSensor").add("MagnetSwitchStatus",
    // false).withWidget(BuiltInWidgets.kBooleanBox).getEntry();

  }

  public void setClimberPose(ClimberSetpoints climberSetpoints) {

    setPoint = climberSetpoints.getValue();
  }

  public double getClimberSetpoint() {

    return setPoint;
  }

  public boolean atSetpoint() {

    System.out.println(climberEncoder.getPosition() - setPoint);
    return Math.abs(climberEncoder.getPosition() - setPoint) <= tolerance;
  }

  public void climberUp() {

    if (setPoint < ClimberSetpoints.UPMAX.getValue() - climberPositionalRateOfChange) {

      setPoint =
          Math.min(setPoint + climberPositionalRateOfChange, ClimberSetpoints.UPMAX.getValue());
    } else {
      setPoint = ClimberSetpoints.UPMAX.getValue();
    }
  }

  public void climberDown() {

    if (setPoint > ClimberSetpoints.ZERO.getValue() + climberPositionalRateOfChange) {
      setPoint =
          Math.max(setPoint - climberPositionalRateOfChange, ClimberSetpoints.ZERO.getValue());
    } else {
      setPoint = ClimberSetpoints.ZERO.getValue();
    }
  }

  public void climberMove(double lkajfds) {

    double newSetPoint = setPoint + lkajfds;

    if (newSetPoint > ClimberSetpoints.ZERO.getValue()
        && newSetPoint < ClimberSetpoints.UPMAX.getValue()) {

      setPoint = newSetPoint;
    }
  }

  @Override
  public LoggableInputs log() {
    climberLogAutoLogged.position = climberEncoder.getPosition();
    climberLogAutoLogged.setpoint = setPoint;
    return climberLogAutoLogged;
  }

  @Override
  public void periodic() {
    climberPositionGenericEntry.setDouble(setPoint);
    climberActualPositionGenericEntry.setDouble(climberEncoder.getPosition());

    if (!pitZero) {
      climberPID.setReference(setPoint, ControlType.kPosition);
      climberPID2.setReference(setPoint, ControlType.kPosition);
    }

    /*
     * digiput.setDouble(digitalInput.get()?1:0);
     * digiputLimit.setBoolean(m_reverseLimit.isPressed());
     */

  }

  public void zeroClimber() {
    climberMotor1.set(-0.1);
  }

  public void stop() {
    double climberSpeed = 0.0;
    climberMotor1.set(climberSpeed);
    climberMotor2.set(climberSpeed);
  }

  public void toggleTarget() {

    if (targetClimb == null) {

      targetClimb = ClimberSetpoints.UPMAX;

    } else if (targetClimb == ClimberSetpoints.UPMAX) {

      targetClimb = ClimberSetpoints.CLIMBED;

    } else if (targetClimb == ClimberSetpoints.CLIMBED) {

      targetClimb = ClimberSetpoints.UPMAX;
    }
    setPoint = targetClimb.getValue();
  }

  public void pitDownRight() {
    pitZero = true;
    climberMotor2.enableSoftLimit(SoftLimitDirection.kReverse, false);
    climberMotor2.set(-pitRate);
  }

  public void pitUpRight() {
    pitZero = true;
    climberMotor2.set(pitRate);
  }

  public void pitDownLeft() {
    pitZero = true;
    climberMotor1.enableSoftLimit(SoftLimitDirection.kReverse, false);
    climberMotor1.set(-pitRate);
  }

  public void pitUpLeft() {
    pitZero = true;
    climberMotor1.set(pitRate);
  }

  public void pitStopRight() {
    pitZero = false;
    climberMotor2.set(0);
    climberEncoder2.setPosition(0);
    climberMotor2.enableSoftLimit(SoftLimitDirection.kReverse, true);
  }

  public void pitStopLeft() {
    pitZero = false;
    climberMotor1.set(0);
    climberEncoder.setPosition(0);
    climberMotor1.enableSoftLimit(SoftLimitDirection.kReverse, true);
  }
}
