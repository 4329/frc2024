package frc.robot.utilities;

public enum ShotRpms {
  PASS(3000),
  REV(3300);

  private int value;

  private ShotRpms(int adsf) {

    this.value = adsf;
  }

  public int getValue() {

    return value;
  }
}
