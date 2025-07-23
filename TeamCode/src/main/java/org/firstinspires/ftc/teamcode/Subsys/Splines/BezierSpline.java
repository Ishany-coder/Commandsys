package org.firstinspires.ftc.teamcode.Subsys.Splines;

import android.util.Log;

import com.arcrobotics.ftclib.geometry.Pose2d;
import com.arcrobotics.ftclib.geometry.Rotation2d;
import com.arcrobotics.ftclib.geometry.Vector2d;
import com.qualcomm.robotcore.hardware.Gamepad;

import org.firstinspires.ftc.teamcode.Robot.RobotConstants;
import org.firstinspires.ftc.teamcode.Subsys.Drive;
import org.firstinspires.ftc.teamcode.Subsys.utils.Controllers.FeedForward.FeedforwardController;
import org.firstinspires.ftc.teamcode.Subsys.utils.Controllers.PurePursuit.PurePursuitController;
import org.firstinspires.ftc.teamcode.Subsys.utils.Controllers.SquID.DrivetrainSquIDController;
import org.firstinspires.ftc.teamcode.Subsys.utils.GoBildaPinpoint;
import org.firstinspires.ftc.teamcode.Subsys.utils.Point;

import java.util.ArrayList;
import java.util.Collections;
import java.util.List;

public class BezierSpline extends Spline{
    private final RobotConstants robotConstants = new RobotConstants();
    private final Drive drive;
    private final DrivetrainSquIDController squid;
    private final GoBildaPinpoint odo;
    public BezierSpline(Drive drive, DrivetrainSquIDController squid, GoBildaPinpoint odo) {
        this.drive = drive;
        this.squid = squid;
        this.odo = odo;
    }
    // take an input of 2 control points for a cubic bezier
    @Override
    public List<Pose2d> generateSpline(Pose2d startPose, List<Pose2d> ControlPoints, Pose2d endPose) {
        int numPoints = 20; //get 20 points
        Log.i("MOVING ROBOT: ", "BEZIER MOVEMENT");
        Vector2d P0 = new Vector2d(startPose.getX(), startPose.getY());
        Vector2d P3 = new Vector2d(endPose.getX(), endPose.getY());
        Vector2d P1 = new Vector2d(ControlPoints.get(0).getX(), ControlPoints.get(0).getY());
        Vector2d P2 = new Vector2d(ControlPoints.get(1).getX(), ControlPoints.get(1).getY());

        List<Pose2d> curvePoses = new ArrayList<>();
        Log.i("INITIALIZED: ", "CONTROL POINTS");
        for (int i = 0; i <= numPoints; i++) {
            double t = (double) i / numPoints;
            double oneMinusT = 1 - t;
            Log.i("GOT T value: ", "T: " + t);
            // Curve position
            double x = Math.pow(oneMinusT, 3) * P0.getX()
                    + 3 * Math.pow(oneMinusT, 2) * t * P1.getX()
                    + 3 * oneMinusT * Math.pow(t, 2) * P2.getX()
                    + Math.pow(t, 3) * P3.getX();
            Log.i("GOT X value: ", "X: " + x);
            double y = Math.pow(oneMinusT, 3) * P0.getY()
                    + 3 * Math.pow(oneMinusT, 2) * t * P1.getY()
                    + 3 * oneMinusT * Math.pow(t, 2) * P2.getY()
                    + Math.pow(t, 3) * P3.getY();
            Log.i("GOT Y value: ", "Y: " + y);
            // Derivative (tangent) for heading
            double dx = 3 * Math.pow(oneMinusT, 2) * (P1.getX() - P0.getX())
                    + 6 * oneMinusT * t * (P2.getX() - P1.getX())
                    + 3 * Math.pow(t, 2) * (P3.getX() - P2.getX());
            Log.i("GOT DX value: ", "DX: " + dx);
            double dy = 3 * Math.pow(oneMinusT, 2) * (P1.getY() - P0.getY())
                    + 6 * oneMinusT * t * (P2.getY() - P1.getY())
                    + 3 * Math.pow(t, 2) * (P3.getY() - P2.getY());
            Log.i("GOT DY value: ", "DY: " + dy);
            double headingRadians = Math.atan2(dy, dx);
            double headingDegrees = Math.toDegrees(headingRadians);

            curvePoses.add(new Pose2d(x, y, Rotation2d.fromDegrees(headingDegrees)));
            Log.i("ADDED NEW POSE: ", "X: " + x + " Y: " + y + " Heading: " + headingDegrees + " degrees");
        }
        return curvePoses;
    }

    @Override
    public void moveSpline(List<Pose2d> path) {
        if (path.isEmpty()) return;

        Pose2d prevPose = path.get(0);
        long prevTime = System.currentTimeMillis();
        FeedforwardController ff = new FeedforwardController(robotConstants.FeedForwardKV, robotConstants.FeedForwardKA);
        Log.i("MOVING ROBOT", "INITIALIZED");

        double currentVel = 0;
        double RemainingDist = 0;
        for (int i = 0; i < path.size() - 1; i++) {
            Pose2d expectedPose = path.get(i);
            Pose2d currentPose = new Pose2d(odo.getPosX(), odo.getPosY(), new Rotation2d(odo.getHeading())); // in Inches

            // Calculate dt
            long currentTime = System.currentTimeMillis();
            double dt = (currentTime - prevTime) / 1000.0; // seconds
            prevTime = currentTime;

            // Calculate current velocity vector from odometry
            Pose2d velocityVector = new Pose2d(
                    (currentPose.getX() - prevPose.getX()) / dt,
                    (currentPose.getY() - prevPose.getY()) / dt,
                    new Rotation2d((currentPose.getHeading() - prevPose.getHeading()) / dt)
            );

            // Error check
            double dx = expectedPose.getX() - currentPose.getX();
            double dy = expectedPose.getY() - currentPose.getY();
            double distanceError = Math.hypot(dx, dy);

            // If off path, use SquID to correct
            if (distanceError > 0.5) {
                int start = Math.max(0, i - 10);
                int end = Math.min(path.size(), i + 10);

                Pose2d closestPose = path.get(i);
                double closestDistance = currentPose.getTranslation().getDistance(closestPose.getTranslation());

                for (int j = start; j < end; j++) {
                    Pose2d candidate = path.get(j);
                    double dist = currentPose.getTranslation().getDistance(candidate.getTranslation());
                    if (dist < closestDistance) {
                        closestPose = candidate;
                        closestDistance = dist;
                    }
                }

                Pose2d correction = squid.calculate(closestPose, currentPose, velocityVector);
                drive.setTarget(new Point(correction.getX() - currentPose.getX(), correction.getY() - currentPose.getY(), correction.getHeading()));
                drive.DriveToTarget();
            }

            // Get lookahead point
            PurePursuitController controller = new PurePursuitController(robotConstants.PurePursuitLookahead);
            Pose2d lookahead = controller.getLookaheadPoint(path, currentPose);
            Log.i("LOOKAHEAD POINT", "X: " + lookahead.getX() + " Y: " + lookahead.getY());

            // Get motion vector using SquID
            Pose2d movement = squid.calculate(lookahead, currentPose, velocityVector);
            // Compute acceleration estimates
            double ax = (velocityVector.getX() - (currentPose.getX() - prevPose.getX()) / dt) / dt;
            double ay = (velocityVector.getY() - (currentPose.getY() - prevPose.getY()) / dt) / dt;

            // Feedforward power commands
            double powerX = ff.calculate(movement.getX(), ax);
            double powerY = ff.calculate(movement.getY(), ay);
            double velocity = Math.hypot(dx, dy);
            // Normalize and scale to current velocity
            // norm linear dist left
            double norm = Math.hypot(powerX, powerY);
            double ffx = 0;
            double ffy = 0;
            if (norm > 0.01) {
                ffx = (powerX / norm) * velocity;
                ffy = (powerY / norm) * velocity;
            }
            drive.setTarget(new Point(ffx + movement.getX(), ffy + movement.getY(), movement.getHeading()));
            drive.DriveToTarget();
            prevPose = currentPose;
        }
        Log.i("MOVING ROBOT", "PURE PURSUIT FINISHED");
    }
}
