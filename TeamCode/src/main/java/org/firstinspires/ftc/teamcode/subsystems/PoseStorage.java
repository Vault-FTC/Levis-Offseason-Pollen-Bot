package org.firstinspires.ftc.teamcode.subsystems;

import org.firstinspires.ftc.robotcore.external.navigation.Pose2D;

/**
 * Hands the robot's field pose from autonomous to teleop. Static fields survive between
 * OpModes as long as the Robot Controller app keeps running.
 */
public class PoseStorage {
    /** Last known field pose written by autonomous, or null if there is none. */
    public static Pose2D currentPose = null;
}
