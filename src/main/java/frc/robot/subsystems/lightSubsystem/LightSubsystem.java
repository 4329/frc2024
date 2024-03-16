package frc.robot.subsystems.lightSubsystem;

import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj.util.Color8Bit;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import java.awt.Color;
import java.util.ArrayList;
import java.util.List;
import java.util.Optional;

public class LightSubsystem extends SubsystemBase {

  private LightIO lightIO;

  private int hue;
  private int saturation = 255;
  private int brightness = 128;

  private List<Color8Bit> savedColors;

  public LightSubsystem(LightIO lightIndividualSubsystemIO) {
    this.lightIO = lightIndividualSubsystemIO;

    savedColors = new ArrayList<>(lightIndividualSubsystemIO.getLength());

    Optional<Alliance> alliance = DriverStation.getAlliance();
    if (alliance.isPresent()) {
      if (alliance.get().equals(DriverStation.Alliance.Red)) {
        hue = 0;
        System.out.println(
            "000000000000000000000000000000000000000000000000000000000000000000000000");
      } else {
        hue = 120;
        System.out.println("120........................................................");
      }
    } else {
      hue = 60;
      System.out.println("60%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%");
    }
  }

  public void beforeMatchColors() {
    for (int i = 0; i < lightIO.getLength(); i++) {
      lightIO.setHSV(i, hue, 255, 150);
    }
  }

  public void blank() {
    brightness = 0;
    for (int i = 0; i < lightIO.getLength(); i++) {
      Color8Bit color = lightIO.getLED8Bit(i);
      savedColors.add(i, color);
      lightIO.setHSV(i, 0, 0, brightness);
    }
  }

  public void lightsOn() {
    brightness = 128;
    for (int i = 0; i < lightIO.getLength(); i++) {
      Color8Bit color = savedColors.get(i);
      if (color != null) {
        float[] hsvVals = Color.RGBtoHSB(color.red, color.green, color.blue, null);
        lightIO.setHSV(i, (int) (hsvVals[0] * 180.0), (int) (hsvVals[1] * 255.0), brightness);
      } else {
        lightIO.setHSV(i, 0, 0, brightness);
      }
    }
  }

  public void progressBarColorrrr(double progress, int hue, int saturation) {
    int length = (int) (lightIO.getLength() * progress);
    for (int i = 0; i <= length; i++) {
      lightIO.setHSV(i, 60, saturation, 150);
    }
  }

  public void loadingBarColor(int start, int stop, int hue, int saturation) {
    for (int i = 0; i < lightIO.getLength(); i++) {
      if (i > start && i < stop) {
        lightIO.setHSV(i, hue, saturation, 150);
      } else {
        lightIO.setHSV(i, 0, 0, 0);
      }
    }
  }

  public void setColor(int hue, int saturation) {
    for (int i = 0; i < lightIO.getLength(); i++) {
      lightIO.setHSV(i, hue, saturation, brightness);
    }
  }

  public void setBlack() {
    for (int i = 0; i < lightIO.getLength(); i++) {
      lightIO.setHSV(i, 0, 0, 0);
    }
  }

  public void rainbow() {
    // For every pixel
    for (var i = 0; i < lightIO.getLength(); i++) {
      // Calculate the hue - hue is easier for rainbows because the color
      // shape is a circle so only one value needs to precess
      final int hues = (int) (hue + (i * 180 / lightIO.getLength())) % 180;
      // Set the value
      lightIO.setHSV(i, hues, 255, 128);
    }
    // Increase by to make the rainbow "move"
    hue += 1;
    // Check bounds
    hue %= 180;
  }

  @Override
  public void periodic() {
    lightIO.periodic();

    rainbow();
  }
}
