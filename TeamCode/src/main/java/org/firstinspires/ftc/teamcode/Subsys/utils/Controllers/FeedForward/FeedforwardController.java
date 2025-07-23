package org.firstinspires.ftc.teamcode.Subsys.utils.Controllers.FeedForward;

public class FeedforwardController {

    private final double kV;
    private final double kA;

    public FeedforwardController(double kV, double kA) {
        this.kV = kV;
        this.kA = kA;
    }

    public double calculate(double velocity, double acceleration) {
        return kV * velocity + kA * acceleration;
    }

    public double getKV() {
        return kV;
    }

    public double getKA() {
        return kA;
    }
}