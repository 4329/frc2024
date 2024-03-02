package frc.robot.utilities;

import java.util.Optional;

import javax.swing.text.html.Option;

import java.sql.Driver;

import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;

public class AprilTagUtil {

    public static int getAprilTagSpeakerIDAprilTagIDSpeaker() {

        Optional<Alliance> allly = DriverStation.getAlliance();

        if (allly.isPresent() && allly.get().equals (DriverStation.Alliance.Red)) {
            
            return 4;


        } else {
            return 7;
        }
            
            
    }
}
