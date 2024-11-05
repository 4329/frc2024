package frc.robot.utilities;

public enum ShotRpms {
  AMP(500),
  PASS(3950),
  REV(3300);

  private int value;

  private ShotRpms(int adsf) {

    this.value = adsf;
  }

  public int getValue() {

    return value;
  }
}
