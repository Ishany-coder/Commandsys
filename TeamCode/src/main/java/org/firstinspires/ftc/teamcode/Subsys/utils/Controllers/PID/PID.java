package org.firstinspires.ftc.teamcode.Subsys.utils.Controllers.PID;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.util.ElapsedTime;

@Config
public class PID {
    private double kP, kI, kD;
    private double errorSum = 0;
    private double lastError = 0;
    private double outputMin = -1;
    private double outputMax = 1;
    private double tolerance = 0.01;

    private final ElapsedTime timer = new ElapsedTime();

    public PID(double kP, double kI, double kD) {
        setPID(kP, kI, kD);
        timer.reset();
    }

    public void setPID(double kP, double kI, double kD) {
        this.kP = kP;
        this.kI = kI;
        this.kD = kD;
    }

    public void setOutputLimits(double min, double max) {
        this.outputMin = min;
        this.outputMax = max;
    }

    public void setTolerance(double tolerance) {
        this.tolerance = tolerance;
    }

    public double calculate(double setpoint, double current) {
        double error = setpoint - current;
        double deltaTime = timer.seconds();
        timer.reset();

        if (deltaTime <= 0) deltaTime = 1e-3;

        errorSum += error * deltaTime;
        double dError = (error - lastError) / deltaTime;

        double output = kP * error + kI * errorSum + kD * dError;
        output = clamp(output, outputMin, outputMax);

        lastError = error;

        return output;
    }

    public boolean atSetpoint(double setpoint, double current) {
        return Math.abs(setpoint - current) < tolerance;
    }

    public void reset() {
        errorSum = 0;
        lastError = 0;
        timer.reset();
    }

    private double clamp(double value, double min, double max) {
        return Math.max(min, Math.min(max, value));
    }
}