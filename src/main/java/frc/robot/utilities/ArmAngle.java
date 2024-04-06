package frc.robot.utilities;

public enum ArmAngle {
  ZERO(0),
  SUB(0),
  HORIZONTAL(12.5),
  INTAKE(1),
  PASS(3),
  SUBWOOF(1),
  INDEXSOURCE(12.5),
  SHOOTERSOURCE(8.3333),
  AMPDEX(9.547),
  SHOOTERARMAMP(12.5),
  FULL(18);

  private double value;

  private ArmAngle(double bob) {

    this.value = bob;
  }

  public double getValue() {

    return value;
  }
}
