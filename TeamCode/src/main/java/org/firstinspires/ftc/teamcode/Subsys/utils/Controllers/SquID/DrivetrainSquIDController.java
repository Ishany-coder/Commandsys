package org.firstinspires.ftc.teamcode.Subsys.utils.Controllers.SquID;

import com.acmerobotics.dashboard.config.Config;
import com.arcrobotics.ftclib.geometry.Pose2d;
import com.arcrobotics.ftclib.geometry.Rotation2d;
import com.arcrobotics.ftclib.geometry.Transform2d;
import com.arcrobotics.ftclib.geometry.Translation2d;
import com.qualcomm.robotcore.util.ElapsedTime;

@Config
public class DrivetrainSquIDController {
    // Tuning parameter to adjust how loop time affects velocity-based distance calculation
    public static double looptimeAdjuster = 15;

    private final ElapsedTime loopTime;  // Timer to track loop iteration duration
    private final  SquIDController squid; // Custom PID controller for movement adjustments

    // Constructor: Initializes the PID controller and loop timer
    public DrivetrainSquIDController() {
        squid = new SquIDController();
        loopTime = new ElapsedTime();
    }

    /**
     * Calculates the necessary movement correction for the drivetrain based on target and current pose.
     * This method compensates for motion delay by estimating the robot's movement due to velocity.
     *
     * @param targetPose The desired position of the robot
     * @param currentPose The actual current position of the robot
     * @param currentVelocity The current velocity of the robot
     * @return A corrected Pose2d that accounts for motion delay
     */
    public Pose2d calculate(Pose2d targetPose, Pose2d currentPose, Pose2d currentVelocity) {
        // Compute the magnitude (length) of the velocity vector
        double currentVelMag = currentVelocity.getTranslation().getNorm();

        // Calculate the time elapsed in the control loop
        double deltaSeconds = loopTime.seconds();

        // Adjust velocity magnitude based on loop time
        currentVelMag *= deltaSeconds;

        // Determine the direction of movement using atan2
        double velAngle = Math.atan2(currentVelocity.getY(), currentVelocity.getX());

        // Estimate how far the robot will travel due to velocity
        double distance = Math.max(getDistanceFromVelocity(currentVelMag), 0);

        // Adjust current position by adding the predicted travel distance in the velocity direction
        currentPose = currentPose.plus(new Transform2d(
                new Translation2d(Math.cos(velAngle) * distance, Math.sin(velAngle) * distance),
                new Rotation2d()
        ));

        // Calculate the remaining distance to the target
        double magnitude = currentPose.getTranslation().getDistance(targetPose.getTranslation());

        // Use the PID controller to generate an adjustment based on the error
        magnitude = squid.calculate(magnitude, 0);

        // Compute the direction to the target
        Translation2d delta = targetPose.getTranslation().minus(currentPose.getTranslation());
        double posAngle = Math.atan2(delta.getY(), delta.getX());

        // Reset the loop timer for the next iteration
        loopTime.reset();

        // Return a new Pose2d with the computed movement correction
        return new Pose2d(
                Math.cos(posAngle) * magnitude,  // X component
                Math.sin(posAngle) * magnitude,  // Y component
                new Rotation2d()                 // Rotation remains unchanged
        );
    }

    /**
     * Estimates how far the robot will travel based on its velocity.
     * Uses a quadratic regression model to predict distance traveled per control loop.
     *
     * @param velocity The current velocity magnitude
     * @return The estimated travel distance
     */
    private static double getDistanceFromVelocity(double velocity) {
        velocity *= looptimeAdjuster;  // Adjust velocity based on tuning constant
        // Quadratic regression equation to approximate travel distance
        return 0.00286 * velocity * velocity + 0.304 * velocity - 0.837;
    }
}