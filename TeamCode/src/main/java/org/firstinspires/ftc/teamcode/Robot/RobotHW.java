package org.firstinspires.ftc.teamcode.Robot;

import com.qualcomm.hardware.lynx.LynxModule;
import com.qualcomm.robotcore.hardware.HardwareMap.DeviceMapping;
import com.qualcomm.robotcore.hardware.VoltageSensor;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.Subsys.Wsubsystem;

import java.util.List;

public class RobotHW extends Wsubsystem {
    private final Telemetry telemetry;
    private final List<LynxModule> allHubs;
    private final DeviceMapping<VoltageSensor> voltageSensor;
    public static double voltage = 0.0;
    private ElapsedTime voltageTimer = new ElapsedTime();
    private double loopTime = 0.0;
    private double prevLoopTime = 0.0;

    public RobotHW(Telemetry telemetry, List<LynxModule> allHubs, DeviceMapping<VoltageSensor> voltageSensor) {
        this.telemetry = telemetry;
        this.allHubs = allHubs;
        this.voltageSensor = voltageSensor;
        for(LynxModule hub : allHubs) {
            hub.setBulkCachingMode(LynxModule.BulkCachingMode.MANUAL);
        }
        voltage = voltageSensor.iterator().next().getVoltage();
    }

    @Override
    public void read() {
        if(voltageTimer.seconds() > 1) { // check every second
            voltage = voltageSensor.iterator().next().getVoltage();
            voltageTimer.reset();
        }
        loopTime = System.nanoTime();
    }

    @Override
    public void loop() {
        loopTime = 1e9 / (loopTime - prevLoopTime);
        telemetry.addData("FPS: ", loopTime);
        prevLoopTime = loopTime;
    }

    @Override
    public void write() {
        for(LynxModule hub : allHubs){
            hub.clearBulkCache();
        }
    }
}
