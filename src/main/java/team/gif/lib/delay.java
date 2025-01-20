package team.gif.lib;

public enum delay {
    DELAY_0(0),
    DELAY_1(1),
    DELAY_2(2),
    DELAY_3(3),
    DELAY_4(4),
    DELAY_5(5),
    DELAY_6(6),
    DELAY_7(7),
    DELAY_8(8),
    DELAY_9(9),
    DELAY_10(10),
    DELAY_11(11),
    DELAY_12(12),
    DELAY_13(13),
    DELAY_14(14),
    DELAY_15(15);

    private double value;

    delay(double value) {
        this.value = value;
    }

    public double getValue() {
        return this.value;
    }
}