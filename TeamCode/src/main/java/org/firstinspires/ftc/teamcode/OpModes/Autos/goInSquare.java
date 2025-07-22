package org.firstinspires.ftc.teamcode.OpModes.Autos;

import com.arcrobotics.ftclib.command.CommandBase;
import com.arcrobotics.ftclib.command.CommandOpMode;
import com.arcrobotics.ftclib.command.CommandScheduler;
import com.arcrobotics.ftclib.command.RunCommand;
import com.arcrobotics.ftclib.command.SequentialCommandGroup;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.Robot.Robot;
import org.firstinspires.ftc.teamcode.Subsys.Commands.DriveCommand;
import org.firstinspires.ftc.teamcode.Subsys.utils.Point;

@Autonomous
// example opMode which uses the command based sequence made before
public class goInSquare extends CommandOpMode {
    private Robot robot;
    @Override
    public void initialize() {
        robot = new Robot(telemetry, hardwareMap, Robot.opModeType.AUTO);
        robot.drive.setLocation(new Point());
        robot.drive.recalibrateIMU();
        robot.schedule(
                new RunCommand(() -> robot.read()),
                new RunCommand(() -> robot.loop()),
                new RunCommand(() -> robot.write()),
                new RunCommand(() -> robot.drive.DriveToTarget()),
                new SequentialCommandGroup(
                        new DriveCommand.DriveToPoint(robot.drive, new Point(24, 0, 0)),
                        new DriveCommand.TurnToHeading(robot.drive, 90),
                        new DriveCommand.DriveToPoint(robot.drive, new Point(24, 24, 90)),
                        new DriveCommand.TurnToHeading(robot.drive, 180),
                        new DriveCommand.DriveToPoint(robot.drive, new Point(0, 24, 180)),
                        new DriveCommand.TurnToHeading(robot.drive, 270),
                        new DriveCommand.DriveToPoint(robot.drive, new Point(0, 0, 270)),
                        new DriveCommand.TurnToHeading(robot.drive, 0)
                )
        );
    }
}
