package org.firstinspires.ftc.teamcode.Subsys;

import com.arcrobotics.ftclib.command.SubsystemBase;

public abstract class Wsubsystem extends SubsystemBase {
    abstract public void read();
    abstract public void loop();
    abstract public void write();
}
