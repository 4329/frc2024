package frc.robot.utilities;

import edu.wpi.first.wpilibj.DriverStation;

public class AprilTagUtil {

    public static int getAprilTagSpeakerIDAprilTagIDSpeaker() {

        if (DriverStation.Alliance.Red.equals(DriverStation.getAlliance().get()))
            return 4;
        return 7;

    }
}
