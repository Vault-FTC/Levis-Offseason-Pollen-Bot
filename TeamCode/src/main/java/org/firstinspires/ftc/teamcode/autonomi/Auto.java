package org.firstinspires.ftc.teamcode.autonomi;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.Pose2D;
import org.firstinspires.ftc.teamcode.FieldConstants;
import org.firstinspires.ftc.teamcode.subsystems.Drivebase;
import org.firstinspires.ftc.teamcode.subsystems.PoseStorage;

@Autonomous
public class Auto extends LinearOpMode {
    Drivebase drivebase;

    @Override
    public void runOpMode() throws InterruptedException {
        // Hardware must be looked up here, not in field initializers: hardwareMap is null until runOpMode.
        drivebase = new Drivebase(hardwareMap);

        // Put the odometry in the shared field frame (inches, field center at (70, 70)).
        drivebase.getPinpoint().resetPosAndIMU();
        sleep(300);   // let the Pinpoint finish its IMU reset before we write a pose to it
        Pose2D start = FieldConstants.START_POSE;
        drivebase.setCurrentPose(start);
        PoseStorage.currentPose = start;

        // Target in field coordinates. This holds the starting pose, like the old (0, 0, 0) target did.
        Location target = new Location(
                start.getX(FieldConstants.DISTANCE_UNIT),
                start.getY(FieldConstants.DISTANCE_UNIT),
                start.getHeading(AngleUnit.DEGREES));

        waitForStart();

        while (opModeIsActive()) {
            drivebase.update();   // without this the Pinpoint pose never changes
            PoseStorage.currentPose = drivebase.getPosition();

            if (drivebase.isAtPosition(target, 1, 2)) {
                drivebase.drive(0, 0, 0);
            } else {
                drivebase.driveToPosition(target, telemetry);
            }

            telemetry.addData("Position", drivebase.getPositionTelemetry());
            telemetry.update();
        }

        drivebase.drive(0, 0, 0);
    }
}
