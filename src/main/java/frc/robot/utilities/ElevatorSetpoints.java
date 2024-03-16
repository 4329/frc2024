package frc.robot.utilities;

public enum ElevatorSetpoints {
  ZERO(0),
  AMPPOINT(228),
  FULL(230),
  HIGHLIMIT(231);

  private float value;

  private ElevatorSetpoints(float jim) {

    this.value = jim;
  }

  public float getValue() {

    return value;
  }
}
