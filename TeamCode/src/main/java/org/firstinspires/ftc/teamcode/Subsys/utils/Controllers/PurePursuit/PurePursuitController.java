package org.firstinspires.ftc.teamcode.Subsys.utils.Controllers.PurePursuit;

import com.arcrobotics.ftclib.geometry.Pose2d;
import com.arcrobotics.ftclib.geometry.Translation2d;

import java.util.List;

public class PurePursuitController {

    private double lookaheadDistance;

    public PurePursuitController(double lookaheadDistance) {
        this.lookaheadDistance = lookaheadDistance;
    }

    public void setLookaheadDistance(double distance) {
        this.lookaheadDistance = distance;
    }

    /**
     * @param path List of poses representing the path
     * @param currentPose Robot's current position
     * @return Target point to pursue
     */
    public Pose2d getLookaheadPoint(List<Pose2d> path, Pose2d currentPose) {
        Pose2d closest = null;
        double minDist = Double.MAX_VALUE;

        for (int i = 0; i < path.size(); i++) {
            Pose2d point = path.get(i);
            double dist = currentPose.getTranslation().getDistance(point.getTranslation());

            // Find the first point >= lookahead distance
            if (dist >= lookaheadDistance && dist < minDist) {
                closest = point;
                minDist = dist;
            }
        }

        // If nothing is far enough, return the last point
        return (closest != null) ? closest : path.get(path.size() - 1);
    }

    /**
     * Computes the movement vector (relative to robot) to follow the path.
     * @param path Waypoints to follow
     * @param currentPose Robot's current position
     * @return Movement vector toward lookahead point
     */
    public Translation2d getFollowVector(List<Pose2d> path, Pose2d currentPose) {
        Pose2d lookahead = getLookaheadPoint(path, currentPose);
        double dx = lookahead.getX() - currentPose.getX();
        double dy = lookahead.getY() - currentPose.getY();
        return new Translation2d(dx, dy);
    }

    public boolean isFinished(List<Pose2d> path, Pose2d currentPose, double tolerance) {
        return currentPose.getTranslation().getDistance(path.get(path.size() - 1).getTranslation()) < tolerance;
    }
}