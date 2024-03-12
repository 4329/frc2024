package frc.robot.subsystems.shoot;

import java.awt.geom.Point2D;

import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.units.Measure;
import edu.wpi.first.units.Voltage;
import edu.wpi.first.wpilibj.sysid.SysIdRoutineLog;
import frc.robot.Model.ShootLogAutoLogged;
import frc.robot.utilities.LinearInterpolationTable;

public interface ShootIO {

    public final LinearInterpolationTable shotTable = new LinearInterpolationTable(
                new Point2D.Double(0, 2500),
                new Point2D.Double(1.1, 2500),
                new Point2D.Double(1.2, 2550),
                new Point2D.Double(1.4, 2650),
                new Point2D.Double(1.6, 2700),
                new Point2D.Double(1.794, 2750),
                new Point2D.Double(2, 2800),
                new Point2D.Double(2.2, 2950),
                new Point2D.Double(2.4, 3050),
                new Point2D.Double(2.47, 2900),
                new Point2D.Double(2.6, 3050),
                new Point2D.Double(2.8, 3200),
                new Point2D.Double(3, 3300));

    default void changeSetpoint(double set) {};

    default boolean atSetpoint() {return false;};

    default void shooterDistance(Pose3d pose) {};

    default boolean aboveSetpoint() {return false;};

    default void stop() {};

    default void periodic() {};

    default void setRPM(double rpm) {};

    default void setVoltage(Measure<Voltage> voltage) {};

    default void getData(SysIdRoutineLog sysIdRoutineLog) {};

    default ShootLogAutoLogged log(ShootLogAutoLogged shootLogAutoLogged) {return shootLogAutoLogged;};
}