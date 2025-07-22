package org.firstinspires.ftc.teamcode.Subsys.utils;

public class SQuiDcontroller {
    private double kSQ;
    public SQuiDcontroller(double kSQ){
        this.kSQ = kSQ;
    }

    /**
     * error is generally current - target
     * @param error
     * @return
     */
    public double calculate(double error){
        return Math.sqrt(Math.abs(error * kSQ)) * Math.signum(error); // basic squid controller
    }
    public void setkSQ(double kSQ){
        this.kSQ = kSQ;
    }
}
