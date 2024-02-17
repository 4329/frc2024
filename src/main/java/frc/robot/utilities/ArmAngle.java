package frc.robot.utilities;

public enum ArmAngle {

    ZERO(0), HORIZONTAL(3), INTAKE(0), FULL(6);

    private float value;

    private ArmAngle(float bob) {

        this.value = bob;

    }

    public float getValue() {

        return value;

    }

}
