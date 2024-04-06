package frc.robot.utilities;

public enum ShotRpms {
  PASS(4500),
  REV(3300);

  private int value;

  private ShotRpms(int adsf) {

    this.value = adsf;
  }

  public int getValue() {

    return value;
  }
}
