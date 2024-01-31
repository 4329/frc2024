package frc.robot.subsystems;

import com.revrobotics.CANSparkBase.ControlType;
import com.revrobotics.CANSparkBase.SoftLimitDirection;
import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.SparkLimitSwitch;
import com.revrobotics.SparkPIDController;

import edu.wpi.first.networktables.GenericEntry;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.shuffleboard.BuiltInWidgets;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.utilities.ArmAngle;
import frc.robot.utilities.ElevatorSetpoints;
import frc.robot.utilities.SparkFactory;

public class ElevatorSubsystem extends SubsystemBase {

    private CANSparkMax elevatorMotor1;
    private CANSparkMax elevatorMotor2;
    private RelativeEncoder elevatorEncoder;
    private SparkPIDController elevatorPID;

    private double setPoint;
    private final double tolerance = 0.1;

    private DigitalInput digitalInput;
    GenericEntry digiput;
    GenericEntry digiputLimit;

    private SparkLimitSwitch m_reverseLimit;


    public ElevatorSubsystem() {

        elevatorMotor1 = SparkFactory.createCANSparkMax(Constants.CANIDConstants.elevatorMotor1, false);
        elevatorMotor2 = SparkFactory.createCANSparkMax(Constants.CANIDConstants.elevatorMotor2, false);

        elevatorPID = elevatorMotor1.getPIDController();
        elevatorEncoder = elevatorMotor1.getEncoder();
        elevatorMotor1.enableSoftLimit(SoftLimitDirection.kForward, true);
        elevatorMotor1.enableSoftLimit(SoftLimitDirection.kReverse, true);
        elevatorMotor1.setSoftLimit(SoftLimitDirection.kForward, ElevatorSetpoints.ONEHUNDRED.getValue());
        elevatorMotor1.setSoftLimit(SoftLimitDirection.kReverse, ElevatorSetpoints.ZERO.getValue());

        elevatorMotor2.follow(elevatorMotor1, true);

        elevatorEncoder.setPosition(0);

        elevatorPID.setP(0.1);
        elevatorPID.setI(0);
        elevatorPID.setD(0.1);
        elevatorPID.setFF(0);

        
        // elevatorEncoder.setPositionConversionFactor(1 / Constants.ArmAngleSubsystemConstants.armGearRatio);
        elevatorMotor1.burnFlash();
        elevatorMotor2.burnFlash();

        digitalInput = new DigitalInput(0);
        digiput = Shuffleboard
        .getTab("MagenetSensor")
        .add("MagnetSensor", 1)
        .withWidget(BuiltInWidgets.kGraph)
        .getEntry();

        m_reverseLimit = elevatorMotor1.getReverseLimitSwitch(SparkLimitSwitch.Type.kNormallyClosed);
        m_reverseLimit.enableLimitSwitch(true);
        digiputLimit = Shuffleboard.getTab("MagnetSensor").add("MagnetSwitchStatus", false).withWidget(BuiltInWidgets.kBooleanBox).getEntry();

    }

    public void setElevatorPose(ElevatorSetpoints elevatorSetpoints) {

        setPoint = elevatorSetpoints.getValue();

    }

    public boolean atSetpoint() {

        return Math.abs(elevatorEncoder.getPosition() - setPoint) <= tolerance;

    }

    @Override
    public void periodic() {

    elevatorPID.setReference(setPoint, ControlType.kPosition);
    digiput.setDouble(digitalInput.get()?1:0);
    digiputLimit.setBoolean(m_reverseLimit.isPressed());

    }

    public void zeroElevator() {
        elevatorMotor1.set(-0.1);
    }

}
