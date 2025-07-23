package org.firstinspires.ftc.teamcode.Subsys.TuningOpModes;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.arcrobotics.ftclib.command.CommandOpMode;
import com.arcrobotics.ftclib.command.CommandScheduler;
import com.arcrobotics.ftclib.command.RunCommand;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotorEx;

import org.firstinspires.ftc.teamcode.Robot.RobotConstants;
import org.firstinspires.ftc.teamcode.Subsys.Drive;
import org.firstinspires.ftc.teamcode.Subsys.utils.GoBildaPinpoint;
import org.firstinspires.ftc.teamcode.Subsys.utils.Point;
@TeleOp
@Config
public class TranslationalTuner extends CommandOpMode {
    private final RobotConstants robotConstants = new RobotConstants();
    private Drive drive;
    @Override
    public void initialize() {
        telemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());
        drive = new Drive(
                hardwareMap.get(DcMotorEx.class, "RFmotor"),
                hardwareMap.get(DcMotorEx.class, "RBmotor"),
                hardwareMap.get(DcMotorEx.class, "LFmotor"),
                hardwareMap.get(DcMotorEx.class, "LBmotor"),
                hardwareMap.get(GoBildaPinpoint.class, "goBildaPinpoint"),
                null,
                telemetry
        );
        CommandScheduler.getInstance().schedule(
                new RunCommand(() -> drive.read()),
                new RunCommand(() -> drive.loop()),
                new RunCommand(() -> drive.write())
        );
        drive.setLocation(new Point());
        drive.recalibrateIMU();
    }
    @Override
    public void run(){
        drive.stickInputs(0,0,0);
        if(gamepad1.a){
            drive.setTarget(new Point());
            drive.DriveToTarget();
        }
        else if(gamepad1.b){
            drive.setTarget(new Point(24, 24, 180));
            drive.DriveToTarget();
        }
        drive.tuneTranslational(robotConstants.SQT);
        super.run();
    }
}
