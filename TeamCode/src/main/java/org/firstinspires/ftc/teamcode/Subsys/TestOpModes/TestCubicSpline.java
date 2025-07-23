package org.firstinspires.ftc.teamcode.Subsys.TestOpModes;

import com.arcrobotics.ftclib.command.CommandScheduler;
import com.arcrobotics.ftclib.command.RunCommand;
import com.arcrobotics.ftclib.geometry.Pose2d;
import com.arcrobotics.ftclib.geometry.Rotation2d;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.Robot.Robot;
import org.firstinspires.ftc.teamcode.Subsys.Drive;
import org.firstinspires.ftc.teamcode.Subsys.Splines.CubicSpline;
import org.firstinspires.ftc.teamcode.Subsys.utils.Controllers.SquID.DrivetrainSquIDController;
import org.firstinspires.ftc.teamcode.Subsys.utils.GoBildaPinpoint;
import org.firstinspires.ftc.teamcode.Subsys.utils.Point;

import java.util.ArrayList;
import java.util.List;

@Autonomous
public class TestCubicSpline extends LinearOpMode {
    CubicSpline splineGenerator;
    Robot robot;
    GoBildaPinpoint odo;
    DrivetrainSquIDController squid;
    List<Pose2d> interiorPoints = new ArrayList<>();
    @Override
    public void runOpMode() throws InterruptedException {
        interiorPoints.add(new Pose2d(2,2,new Rotation2d(90)));
        odo = hardwareMap.get(GoBildaPinpoint.class, "odo");
        squid = new DrivetrainSquIDController();
        robot = new Robot(telemetry, hardwareMap, Robot.opModeType.AUTO);
        splineGenerator = new CubicSpline(odo, squid, robot.drive);
        List<Pose2d> path = splineGenerator.generateSpline(new Pose2d(0,0, new Rotation2d(0)), interiorPoints, new Pose2d(10,10, new Rotation2d(0)));
        waitForStart();
        while(opModeIsActive()){
            CommandScheduler.getInstance().schedule(
                    new RunCommand(() -> robot.read()),
                    new RunCommand(() -> robot.loop()),
                    new RunCommand(() -> robot.write()),
                    new RunCommand(() -> splineGenerator.moveSpline(path))
            );
        }
    }
}
