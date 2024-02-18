package frc.robot.utilities;

public enum ElevatorSetpoints {

    ZERO(0), FULL(208);

    private float value;

    private ElevatorSetpoints(float jim) {

        this.value = jim;

    }

    public float getValue() {

        return value;

    }
}
