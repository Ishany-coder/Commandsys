package org.firstinspires.ftc.teamcode.Subsys.Commands;

import com.arcrobotics.ftclib.command.CommandBase;
import com.arcrobotics.ftclib.gamepad.GamepadEx;

import org.firstinspires.ftc.teamcode.Subsys.Drive;

public class DriveCommand {
    public static class StickInputs extends CommandBase {
        private final Drive drive;
        private final GamepadEx gamepadEx;
        public StickInputs(Drive drive, GamepadEx gamepadEx){
            addRequirements(drive);
            this.drive = drive;
            this.gamepadEx = gamepadEx;
        }

        @Override
        public void execute(){
            drive.stickInputs(gamepadEx.getLeftY(), gamepadEx.getLeftX(), gamepadEx.getRightX());
        }
    }
}
