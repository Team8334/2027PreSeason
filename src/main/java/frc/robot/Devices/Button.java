package frc.robot.Devices;

public enum Button {
    A(1),
    B(2),
    X(3),
    Y(4),
    LEFT_BUMPER(5),
    RIGHT_BUMPER(6),
    BACK(7),
    START(8),
    LEFT_STICK(9),
    RIGHT_STICK(10);

    public final int value;

    Button(int value) {
        this.value = value;
    }
}