package frc.robot.utilities;

public enum ClimberSetpoints {
  ZERO(0),
  MAX(-67.75f),
  OTHERMAX(-71.8f);

  private float value;

  private ClimberSetpoints(float sam) {

    this.value = sam;
  }

  public float getValue() {

    return value;
  }
}
