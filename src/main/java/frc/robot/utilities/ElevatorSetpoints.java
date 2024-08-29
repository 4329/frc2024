package frc.robot.utilities;

public enum ElevatorSetpoints {
  ZERO(0),
  INTAKE(10),
  SOURCE(0),
  SUBWOOF(106),
  AMPPOINT(115),
  FULL(109),
  HIGHLIMIT(115);

  private float value;

  private ElevatorSetpoints(float jim) {

    this.value = jim;
  }

  public float getValue() {

    return value;
  }
}
