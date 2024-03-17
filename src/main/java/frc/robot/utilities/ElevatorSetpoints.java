package frc.robot.utilities;

public enum ElevatorSetpoints {
  ZERO(0),
  AMPPOINT(225),
  FULL(226),
  HIGHLIMIT(227);

  private float value;

  private ElevatorSetpoints(float jim) {

    this.value = jim;
  }

  public float getValue() {

    return value;
  }
}
