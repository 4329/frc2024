package frc.robot.utilities;

public enum ArmAngle {

    ZERO(0), HORIZONTAL(3.6), INTAKE(0), ARMAMP(5.6), FULL(6);

    private double value;

    private ArmAngle(double bob) {

        this.value = bob;

    }

    public double getValue() {

        return value;

    }

}
