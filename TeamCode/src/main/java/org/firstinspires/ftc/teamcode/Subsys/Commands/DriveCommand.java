package org.firstinspires.ftc.teamcode.Subsys.Commands;

import com.arcrobotics.ftclib.command.CommandBase;
import com.arcrobotics.ftclib.gamepad.GamepadEx;
import com.qualcomm.robotcore.robocol.Command;

import org.firstinspires.ftc.teamcode.Subsys.Drive;
import org.firstinspires.ftc.teamcode.Subsys.utils.Point;

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
    public static class DriveToPoint extends CommandBase{
        private final Drive drive;
        private final Point Point;
        public DriveToPoint(Drive drive, Point Point){
            addRequirements(drive);
            this.drive = drive;
            this.Point = Point;
        }
        @Override
        public void initialize(){
            drive.trajectoryStartSequence();
            drive.setTarget(Point);
        }
        @Override
        public boolean isFinished(){
            return drive.isAtTarget();
        }
    }
    public static class TurnToHeading extends CommandBase{
        private final Drive drive;
        private final double Heading;
        public TurnToHeading(Drive drive, double Heading){
            addRequirements(drive);
            this.drive = drive;
            this.Heading = Heading;
        }
        @Override
        public void initialize(){
            drive.trajectoryStartSequence();
            drive.setTargetHeading(Heading);
        }
        @Override
        public boolean isFinished(){
            return drive.isAtTarget();
        }
    }
}
