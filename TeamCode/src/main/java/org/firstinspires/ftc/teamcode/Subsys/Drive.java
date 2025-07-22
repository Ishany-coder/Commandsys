package org.firstinspires.ftc.teamcode.Subsys;

import com.arcrobotics.ftclib.geometry.Vector2d;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.teamcode.Robot.Robot;
import org.firstinspires.ftc.teamcode.Subsys.utils.GoBildaPinpoint;

public class Drive extends Wsubsystem{
    private final DcMotorEx RFmotor, RBmotor, LFmotor, LBmotor;
    private final GoBildaPinpoint goBildaPinpoint;
    private double RobotXInches;
    private double RobotYInches;
    private double RobotHeadingRad;
    private double LBPower, LFPower, RFPower, RBPower;
    private double drive, strafe, turn;
    private final Robot robot;
    private final Telemetry telemetry;
    public Drive(DcMotorEx RFmotor, DcMotorEx RBmotor, DcMotorEx LFmotor, DcMotorEx LBmotor, GoBildaPinpoint goBildaPinpoint, Robot robot, Telemetry telemetry){
        this.RFmotor = RFmotor;
        this.RBmotor = RBmotor;
        this.LFmotor = LFmotor;
        this.LBmotor = LBmotor;

        RFmotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        LFmotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        RBmotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        LBmotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        LBmotor.setDirection(DcMotorSimple.Direction.REVERSE);
        LFmotor.setDirection(DcMotorSimple.Direction.REVERSE);

        this.goBildaPinpoint = goBildaPinpoint;
        goBildaPinpoint.setEncoderDirections(GoBildaPinpoint.EncoderDirection.FORWARD, GoBildaPinpoint.EncoderDirection.FORWARD);

        this.robot = robot;
        this.telemetry = telemetry;
    }
    @Override
    public void read() {
        goBildaPinpoint.update();
        RobotXInches = goBildaPinpoint.getPosX();
        RobotYInches = goBildaPinpoint.getPosY();
        RobotHeadingRad = goBildaPinpoint.getHeading();
    }

    @Override
    public void loop() {
        LFPower = drive + strafe + turn;
        LBPower = drive - strafe + turn;
        RFPower = drive + strafe - turn;
        RBPower = drive - strafe - turn;
    }

    @Override
    public void write() {
        LFmotor.setPower(LFPower);
        LBmotor.setPower(LBPower);
        RFmotor.setPower(RFPower);
        RBmotor.setPower(RBPower);
    }
    public void stickInputs(double drive, double strafe, double turn){
        final double minPow = 0.1;
        this.drive = drive;
        this.strafe = strafe;
        this.turn = turn;

        Vector2d powerVec = new Vector2d(this.drive, this.strafe);
        powerVec = powerVec.rotateBy(RobotHeadingRad);
        this.drive = powerVec.getX();
        this.strafe = powerVec.getY();
    }
}
