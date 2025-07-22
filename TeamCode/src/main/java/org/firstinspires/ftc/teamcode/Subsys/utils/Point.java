package org.firstinspires.ftc.teamcode.Subsys.utils;

public class Point {
    public double getHeading(){
        return heading;
    }
    public double getX(){
        return x;
    }
    public double getY(){
        return y;
    }
    public void setHeading(double heading){
        this.heading = heading;
    }
    public void setX(double x){
        this.x = x;
    }
    public void setY(double y) {
        this.y = y;
    }
    private double x, y, heading;

    public Point(double x, double y, double heading) {
        this.x = x;
        this.y = y;
        this.heading = heading;
    }
    public Point(){
        this.x = 0;
        this.y = 0;
        this.heading = 0;
    }
    public void setPoint(double x, double y, double heading){
        this.x = x;
        this.y = y;
        this.heading = heading;
    }
}
