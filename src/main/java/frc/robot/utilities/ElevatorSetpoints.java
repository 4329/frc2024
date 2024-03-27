package frc.robot.utilities;

public enum ElevatorSetpoints {
  ZERO(0),
  AMPPOINT(108),
  FULL(109),
  HIGHLIMIT(110);

  private float value;

  private ElevatorSetpoints(float jim) {

    this.value = jim;
  }

  public float getValue() {

    return value;
  }
}
