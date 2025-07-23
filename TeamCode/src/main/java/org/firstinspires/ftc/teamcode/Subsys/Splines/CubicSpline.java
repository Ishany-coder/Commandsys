package org.firstinspires.ftc.teamcode.Subsys.Splines;

import android.util.Log;

import com.arcrobotics.ftclib.geometry.Pose2d;
import com.arcrobotics.ftclib.geometry.Rotation2d;
import com.arcrobotics.ftclib.geometry.Translation2d;

import org.firstinspires.ftc.teamcode.Subsys.Drive;
import org.firstinspires.ftc.teamcode.Subsys.utils.Controllers.FeedForward.FeedforwardController;
import org.firstinspires.ftc.teamcode.Subsys.utils.Controllers.PurePursuit.PurePursuitController;
import org.firstinspires.ftc.teamcode.Subsys.utils.Controllers.SquID.DrivetrainSquIDController;
import org.firstinspires.ftc.teamcode.Subsys.utils.GoBildaPinpoint;
import org.firstinspires.ftc.teamcode.Subsys.utils.Point;

import java.util.ArrayList;
import java.util.List;

public class CubicSpline extends Spline{
    private final GoBildaPinpoint odo;
    private final DrivetrainSquIDController squid;
    private final Drive drive;
    private double KV = 0.1;
    private double KA = 0.1;
    public CubicSpline(GoBildaPinpoint odo, DrivetrainSquIDController squid, Drive drive){
        this.odo = odo;
        this.squid = squid;
        this.drive = drive;
    }
    @Override
    public List<Point> generateSpline(Pose2d startPose, List<Translation2d> interiorPoints, Pose2d endPose){
        // Combine all points: start, interior, end
        List<Translation2d> points = new ArrayList<>();
        points.add(startPose.getTranslation());
        points.addAll(interiorPoints);
        points.add(endPose.getTranslation());
        int numSamples = interiorPoints.size();

        int pointCount = points.size();
        double[] t = new double[pointCount];
        for (int i = 0; i < pointCount; i++) t[i] = i;

        // Extract x and y coordinates
        double[] xValues = new double[pointCount];
        double[] yValues = new double[pointCount];
        for (int i = 0; i < pointCount; i++) {
            xValues[i] = points.get(i).getX();
            yValues[i] = points.get(i).getY();
        }

        // Compute second derivatives for x and y
        double[] xSecondDerivatives = computeSecondDerivatives(t, xValues);
        double[] ySecondDerivatives = computeSecondDerivatives(t, yValues);

        List<Point> path = new ArrayList<>();

        for (int sample = 0; sample <= numSamples; sample++) {
            double tSample = (pointCount - 1) * sample / (double)numSamples;
            int segmentIndex = Math.min((int)Math.floor(tSample), pointCount - 2);
            double segmentStart = t[segmentIndex];
            double segmentEnd = t[segmentIndex + 1];
            double segmentLength = segmentEnd - segmentStart;

            double weightStart = (segmentEnd - tSample) / segmentLength;
            double weightEnd = (tSample - segmentStart) / segmentLength;

            double xPos = weightStart * xValues[segmentIndex] + weightEnd * xValues[segmentIndex + 1]
                    + ((Math.pow(weightStart, 3) - weightStart) * xSecondDerivatives[segmentIndex]
                    + (Math.pow(weightEnd, 3) - weightEnd) * xSecondDerivatives[segmentIndex + 1])
                    * Math.pow(segmentLength, 2) / 6.0;

            double yPos = weightStart * yValues[segmentIndex] + weightEnd * yValues[segmentIndex + 1]
                    + ((Math.pow(weightStart, 3) - weightStart) * ySecondDerivatives[segmentIndex]
                    + (Math.pow(weightEnd, 3) - weightEnd) * ySecondDerivatives[segmentIndex + 1])
                    * Math.pow(segmentLength, 2) / 6.0;

            double dx = (xValues[segmentIndex + 1] - xValues[segmentIndex]) / segmentLength
                    - (3 * weightStart * weightStart - 1) * segmentLength * xSecondDerivatives[segmentIndex] / 6
                    + (3 * weightEnd * weightEnd - 1) * segmentLength * xSecondDerivatives[segmentIndex + 1] / 6;

            double dy = (yValues[segmentIndex + 1] - yValues[segmentIndex]) / segmentLength
                    - (3 * weightStart * weightStart - 1) * segmentLength * ySecondDerivatives[segmentIndex] / 6
                    + (3 * weightEnd * weightEnd - 1) * segmentLength * ySecondDerivatives[segmentIndex + 1] / 6;

            double heading = Math.atan2(dy, dx);

            path.add(new Point(xPos, yPos, Math.toDegrees(heading)));
        }
        return path;
    }

    private static double[] computeSecondDerivatives(double[] time, double[] values) {
        int numPoints = time.length;
        double[] interval = new double[numPoints - 1];

        for (int i = 0; i < numPoints - 1; i++) {
            interval[i] = time[i + 1] - time[i];
        }

        double[] rhs = new double[numPoints];
        for (int i = 1; i < numPoints - 1; i++) {
            double slopeNext = (values[i + 1] - values[i]) / interval[i];
            double slopePrev = (values[i] - values[i - 1]) / interval[i - 1];
            rhs[i] = 3 * (slopeNext - slopePrev);
        }

        double[] mainDiag = new double[numPoints];
        double[] upperDiag = new double[numPoints];
        double[] lowerDiag = new double[numPoints];
        double[] secondDeriv = new double[numPoints];

        mainDiag[0] = 1;
        for (int i = 1; i < numPoints - 1; i++) {
            mainDiag[i] = 2 * (time[i + 1] - time[i - 1]);
            upperDiag[i] = interval[i];
            lowerDiag[i] = interval[i - 1];
        }
        mainDiag[numPoints - 1] = 1;

        // Forward elimination
        for (int i = 1; i < numPoints; i++) {
            double scale = lowerDiag[i] / mainDiag[i - 1];
            mainDiag[i] -= scale * upperDiag[i - 1];
            rhs[i] -= scale * rhs[i - 1];
        }

        // Back substitution
        secondDeriv[numPoints - 1] = rhs[numPoints - 1] / mainDiag[numPoints - 1];
        for (int i = numPoints - 2; i >= 0; i--) {
            secondDeriv[i] = (rhs[i] - upperDiag[i] * secondDeriv[i + 1]) / mainDiag[i];
        }

        return secondDeriv;
    }

    @Override
    public void moveSpline(List<Pose2d> path) {
        if (path.isEmpty()) return;

        Pose2d prevPose = path.get(0);
        long prevTime = System.currentTimeMillis();
        FeedforwardController ff = new FeedforwardController(KV, KA);
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
            PurePursuitController controller = new PurePursuitController(6);
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
