package frc.robot.subsystems;

import com.revrobotics.CANSparkBase.ControlType;
import com.revrobotics.CANSparkBase.IdleMode;
import com.revrobotics.CANSparkBase.SoftLimitDirection;
import com.revrobotics.SparkAnalogSensor.Mode;

import java.security.spec.EncodedKeySpec;

import org.littletonrobotics.junction.Logger;

import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.SparkAnalogSensor;
import com.revrobotics.SparkLimitSwitch;
import com.revrobotics.SparkPIDController;

import edu.wpi.first.networktables.GenericEntry;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.shuffleboard.BuiltInWidgets;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.Model.Elevator;
import frc.robot.Model.ElevatorAutoLogged;
import frc.robot.utilities.ArmAngle;
import frc.robot.utilities.ElevatorSetpoints;
import frc.robot.utilities.SparkFactory;

public class ElevatorSubsystem extends SubsystemBase {

    private CANSparkMax elevatorMotor1;
    private CANSparkMax elevatorMotor2;
    private RelativeEncoder elevatorEncoder;
    private SparkPIDController elevatorPID;
    private GenericEntry elevatorPositionGenericEntry;
    private GenericEntry elevatorActualPositionGenericEntry;

    ElevatorAutoLogged elevatorAutoLogged;
    private final double tolerance = 0.1;
    private double setPoint = 0;

    // private SparkAnalogSensor sparkAnalogSensor;
    // GenericEntry digiput;
    // GenericEntry digiputLimit;

    private SparkLimitSwitch m_reverseLimit;

    public ElevatorSubsystem() {
        elevatorAutoLogged = new ElevatorAutoLogged();

        elevatorMotor1 = SparkFactory.createCANSparkMax(Constants.CANIDConstants.elevatorMotor1, true);
        elevatorMotor2 = SparkFactory.createCANSparkMax(Constants.CANIDConstants.elevatorMotor2, true);
        elevatorMotor1.setInverted(true);
        elevatorPID = elevatorMotor1.getPIDController();
        elevatorEncoder = elevatorMotor1.getEncoder();
        elevatorMotor1.enableSoftLimit(SoftLimitDirection.kForward, true);
        elevatorMotor1.enableSoftLimit(SoftLimitDirection.kReverse, true);
        elevatorMotor1.setIdleMode(IdleMode.kBrake);
        elevatorMotor2.setIdleMode(IdleMode.kBrake);
        elevatorMotor1.setSoftLimit(SoftLimitDirection.kForward, ElevatorSetpoints.HIGHLIMIT.getValue());
        elevatorMotor1.setSoftLimit(SoftLimitDirection.kReverse, ElevatorSetpoints.ZERO.getValue());
        elevatorMotor1.enableVoltageCompensation(Constants.voltageCompensation);
        elevatorMotor2.enableVoltageCompensation(Constants.voltageCompensation);
        elevatorPositionGenericEntry = Shuffleboard.getTab("Arm Angle").add("Elevator desired pos", 0).getEntry();
        elevatorActualPositionGenericEntry = Shuffleboard.getTab("Arm Angle").add("Elevator Actual pos", 0).getEntry();

        elevatorMotor2.follow(elevatorMotor1, false);

        elevatorEncoder.setPosition(0);

        elevatorPID.setP(0.5);
        elevatorPID.setI(0);
        elevatorPID.setD(0);
        elevatorPID.setFF(0);
        elevatorPID.setOutputRange(-1, 1);

        // elevatorEncoder.setPositionConversionFactor(1 /
        // Constants.ArmAngleSubsystemConstants.armGearRatio);
        elevatorMotor1.burnFlash();
        elevatorMotor2.burnFlash();

        /*
         * digitalInput = new DigitalInput(0);
         * digiput = Shuffleboard
         * .getTab("MagenetSensor")
         * .add("MagnetSensor", 1)
         * .withWidget(BuiltInWidgets.kGraph)
         * .getEntry();
         */

        m_reverseLimit = elevatorMotor1.getReverseLimitSwitch(SparkLimitSwitch.Type.kNormallyClosed);
        m_reverseLimit.enableLimitSwitch(true);
        // digiputLimit = Shuffleboard.getTab("MagnetSensor").add("MagnetSwitchStatus",
        // false).withWidget(BuiltInWidgets.kBooleanBox).getEntry();

    }

    public void setElevatorPose(ElevatorSetpoints elevatorSetpoints) {

        setPoint = elevatorSetpoints.getValue();

    }

    public boolean atSetpoint() {

        return Math.abs(elevatorEncoder.getPosition() - setPoint) <= tolerance;

    }

    public void elevatorUp() {

        if (setPoint < ElevatorSetpoints.FULL.getValue() - 2) {

            setPoint = Math.min(setPoint + 2, ElevatorSetpoints.FULL.getValue());
        }

        else {
            setPoint = ElevatorSetpoints.FULL.getValue();

        }
    }

    public void elevatorDown() {

        if (setPoint > ElevatorSetpoints.ZERO.getValue() + 2) {

            setPoint = Math.max(setPoint - 2, ElevatorSetpoints.ZERO.getValue());
        }

        else {
            setPoint = ElevatorSetpoints.ZERO.getValue();
        }
    }

    public void updateInputs(Elevator elevator) {
        elevator.position = elevatorEncoder.getPosition();
        elevator.setpoint = setPoint;
        Logger.processInputs("George", elevatorAutoLogged);
    }

    @Override
    public void periodic() {
        updateInputs(elevatorAutoLogged);
        elevatorPositionGenericEntry.setDouble(setPoint);
        elevatorActualPositionGenericEntry.setDouble(elevatorEncoder.getPosition());

        System.out.println("elvator actual position is ->>  " + elevatorEncoder.getPosition());
        elevatorPID.setReference(setPoint, ControlType.kPosition);
        /*
         * digiput.setDouble(digitalInput.get()?1:0);
         * digiputLimit.setBoolean(m_reverseLimit.isPressed());
         */

    }

    public void zeroElevator() {
        elevatorMotor1.set(-0.1);
    }

}
