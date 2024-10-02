package frc.robot.Model;

import edu.wpi.first.wpilibj2.command.Command;

import java.util.HashMap;
import java.util.List;
import java.util.Map;

import com.pathplanner.lib.commands.PathPlannerAuto;
import com.pathplanner.lib.path.PathPlannerPath;
import com.pathplanner.lib.path.PathPlannerTrajectory;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.trajectory.Trajectory;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

public class AutoDisp extends SendableChooser<Command> {

    private final Map<Command, String> autoNames;
    private final Field2d field;
    private String lastName;

    public AutoDisp() {
        autoNames = new HashMap<>();
        field = new Field2d();
    }

    @Override
    public void addOption(String key, Command value) {
        autoNames.put(value, key);
        super.addOption(key, value);
    }

    public String getAutoName(Command command) {
        return autoNames.getOrDefault(command, "");
    }

    public void drawPath() {
        String name = getAutoName(getSelected());
        if (name == lastName || name == "") return;

        List<PathPlannerPath> paths = PathPlannerAuto.getPathGroupFromAutoFile(name);
        field.getObject("traj").setTrajectory(accumFromPaths(paths));
        field.getObject("traj1").setTrajectory(accumFromPaths(PathPlannerAuto.getPathGroupFromAutoFile("2NoteFarAnd4")));
        SmartDashboard.putData(field);
        lastName = name;
    }

    private Trajectory accumFromPaths(List<PathPlannerPath> paths) {
        Trajectory accumulator = new Trajectory();
        for (int i = 0; i < paths.size(); i++) {
            accumulator = accumulator.concatenate(
                    pathTrajToTragTraj(
                            paths.get(i).getTrajectory(new ChassisSpeeds(), new Rotation2d())));
        }
        return accumulator;
    }

    private Trajectory pathTrajToTragTraj(PathPlannerTrajectory pathPlannerTrajectory) {
        return new Trajectory(
                pathPlannerTrajectory.getStates().stream()
                        .map(
                                (state) -> new Trajectory.State(
                                        state.timeSeconds,
                                        state.velocityMps,
                                        state.accelerationMpsSq,
                                        new Pose2d(state.positionMeters, state.targetHolonomicRotation),
                                        state.curvatureRadPerMeter))
                        .toList());
    }
}
