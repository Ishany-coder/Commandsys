package org.firstinspires.ftc.teamcode.Subsys.Splines;

import com.arcrobotics.ftclib.geometry.Pose2d;
import com.arcrobotics.ftclib.geometry.Translation2d;

import org.firstinspires.ftc.teamcode.Subsys.utils.Point;

import java.util.List;

public abstract class Spline {
    public abstract List<Pose2d> generateSpline(Pose2d startPose, List<Pose2d> interiorPoints, Pose2d endPose);
    public abstract void moveSpline(List<Pose2d> path);
}
