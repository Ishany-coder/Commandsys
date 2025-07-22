package org.firstinspires.ftc.teamcode.OpModes.Teleops;

import com.arcrobotics.ftclib.command.CommandOpMode;
import com.arcrobotics.ftclib.command.RunCommand;
import com.arcrobotics.ftclib.gamepad.GamepadEx;

import org.firstinspires.ftc.teamcode.Robot.Robot;
import org.firstinspires.ftc.teamcode.Subsys.Commands.DriveCommand;

public class TeleOp extends CommandOpMode {
    private Robot robot;
    private GamepadEx gamePad1;
    @Override
    public void initialize() {
        robot = new Robot(telemetry, hardwareMap, Robot.opModeType.TELEOP);
        gamePad1 = new GamepadEx(gamepad1);

        robot.schedule(new RunCommand(() -> robot.read()),
                new RunCommand(() -> robot.loop()),
                new RunCommand(() -> robot.write())
        );
        robot.drive.setDefaultCommand(new DriveCommand.StickInputs(robot.drive, gamePad1));
    }
}
