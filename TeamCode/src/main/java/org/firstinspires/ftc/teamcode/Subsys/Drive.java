package org.firstinspires.ftc.teamcode.Subsys;

import com.arcrobotics.ftclib.geometry.Vector2d;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.robotcore.external.navigation.Pose2D;
import org.firstinspires.ftc.teamcode.Robot.Robot;
import org.firstinspires.ftc.teamcode.Subsys.utils.GoBildaPinpoint;
import org.firstinspires.ftc.teamcode.Subsys.utils.Point;
import org.firstinspires.ftc.teamcode.Subsys.utils.SQuiDcontroller;

public class Drive extends Wsubsystem{
    private final double TargetConfirmSec = 0.1;
    private final double HEADING_THRESHOLD_DEGREES = 1.0;
    private final double RADIUS_THRESHOLD_INCHES = 3;
    private Vector2d errorVector = new Vector2d();
    private double headingErrorRad;
    private SQuiDcontroller headingContoller = new SQuiDcontroller(0.0);
    private SQuiDcontroller translationalController = new SQuiDcontroller(0.0);

    private Point targetPoint = new Point();
    private final DcMotorEx RFmotor, RBmotor, LFmotor, LBmotor;
    private final GoBildaPinpoint goBildaPinpoint;
    private double RobotXInches;
    private double RobotYInches;
    private double RobotHeadingRad;
    private double LBPower, LFPower, RFPower, RBPower;
    private double drive, strafe, turn;
    private ElapsedTime targetConfirmTimer = new ElapsedTime();
    private ElapsedTime overTimeProtection = new ElapsedTime();
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
    public void DriveToTarget(){
        headingErrorRad = AngleUnit.normalizeRadians(Math.toRadians(targetPoint.getHeading()) - RobotHeadingRad);  // get the error in radians
        errorVector = new Vector2d(targetPoint.getX() - RobotXInches, targetPoint.getY() - RobotYInches);

        //rotate vector
        errorVector = errorVector.rotateBy(-Math.toDegrees(RobotHeadingRad));

        //caclulate powers
        turn = -headingContoller.calculate(headingErrorRad);
        drive = translationalController.calculate(errorVector.getX());
        strafe = translationalController.calculate(errorVector.getY());
    }
    public void tuneHeading(double kSQ){
        headingContoller.setkSQ(kSQ);
        telemetry.addData("Position", RobotHeadingRad);
        telemetry.addData("Target", targetPoint.getHeading());
        telemetry.addData("Error", AngleUnit.normalizeRadians(Math.toRadians(targetPoint.getHeading()) - RobotHeadingRad));
    }
    public void tuneTranslational(double kSQ){
        translationalController.setkSQ(kSQ);
        telemetry.addData("Position", errorVector.magnitude());
        telemetry.addData("Target", 0.0);
        telemetry.addData("Error", AngleUnit.normalizeRadians(Math.toRadians(targetPoint.getHeading()) - RobotHeadingRad));
    }
    public void recalibrateIMU(){
        goBildaPinpoint.recalibrateIMU();
    }
    public void setTarget(Point point){
        targetPoint = point;
    }
    public boolean isInRadius(Point point, double RadiusInches){
        double xDelta = point.getX() - RobotXInches;
        double yDelta = point.getY() - RobotYInches;
        double distance = xDelta * xDelta + yDelta * yDelta;
        return distance <= RadiusInches * RadiusInches;
    }
    public void trajectoryStartSequence(){
        targetConfirmTimer.reset();
        overTimeProtection.reset();
    }
    public void setTargetHeading(double headingDegrees){
        targetPoint.setHeading(headingDegrees);
    }
    public boolean isAtTarget() {
        if (!isInRadius(targetPoint, RADIUS_THRESHOLD_INCHES) || Math.abs(Math.toDegrees(headingErrorRad)) >= HEADING_THRESHOLD_DEGREES) {
            targetConfirmTimer.reset();
        }
        return (targetConfirmTimer.seconds() >= TargetConfirmSec) || overTimeProtection.seconds() > 3.0;
    }
    public void setLocation(Point point){
        goBildaPinpoint.setPosition(new Pose2D(DistanceUnit.INCH, point.getX(), point.getY(), AngleUnit.RADIANS, point.getHeading()));
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
