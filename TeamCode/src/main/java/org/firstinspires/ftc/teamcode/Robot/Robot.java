package org.firstinspires.ftc.teamcode.Robot;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.qualcomm.hardware.lynx.LynxModule;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.VoltageSensor;

import org.firstinspires.ftc.onbotjava.handlers.file.TemplateFile;
import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.Subsys.Drive;
import org.firstinspires.ftc.teamcode.Subsys.utils.GoBildaPinpoint;

public class Robot extends com.arcrobotics.ftclib.command.Robot {
    private final Telemetry telemetry;
    private final HardwareMap hwMap;
    private final RobotHW robotHW;
    public Drive drive;

    public static enum opModeType{ TELEOP, AUTO }
    public static opModeType opMode = opModeType.TELEOP; // at the start assume Teleop

    public Robot(Telemetry telemetry, HardwareMap hwMap, opModeType opModeType) {
        this.telemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry()); // setup telemetery for FTC dashboard
        this.hwMap = hwMap;
        this.opMode = opModeType;
        robotHW = new RobotHW(
                telemetry,
                hwMap.getAll(LynxModule.class),
                hwMap.voltageSensor
        );

        init();
    }
    public void init(){
        //Add subsystem constructers here
        drive = new Drive(hwMap.get(DcMotorEx.class, "RFmotor"),
                hwMap.get(DcMotorEx.class, "RBmotor"),
                hwMap.get(DcMotorEx.class, "LFmotor"),
                hwMap.get(DcMotorEx.class, "LBmotor"),
                hwMap.get(GoBildaPinpoint.class, "pinpoint"),
                this,
                telemetry);
    }
    public void read(){
        drive.read();
        robotHW.read();
    }
    public void loop(){
        drive.loop();
        robotHW.loop();
    }
    public void write(){
        drive.write();
        robotHW.write(); // happen last so everything finish before bolt cachin
        telemetry.update();
    }
}
