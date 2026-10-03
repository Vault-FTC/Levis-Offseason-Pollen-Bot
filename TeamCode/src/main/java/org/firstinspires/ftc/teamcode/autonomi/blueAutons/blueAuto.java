package org.firstinspires.ftc.teamcode.autonomi.blueAutons;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.robotcore.external.navigation.Pose2D;
import org.firstinspires.ftc.teamcode.FieldConstants;
import org.firstinspires.ftc.teamcode.autonomi.Location;
import org.firstinspires.ftc.teamcode.subsystems.Drivebase;
import org.firstinspires.ftc.teamcode.subsystems.Intake;
import org.firstinspires.ftc.teamcode.subsystems.PoseStorage;
import org.firstinspires.ftc.teamcode.subsystems.Shooter;

/**
 * Shoots from the starting spot with both shooters, then moves toward -X (field left) to leave.
 * All positions are in the shared field frame (inches, field center at (70, 70)); see FieldConstants.
 */
@Autonomous
public class blueAuto extends LinearOpMode {
    static final double POLLEN_SPEED = 1225;
    static final double NECTAR_SPEED = 1215;

    static final double SPIN_UP_TIMEOUT_S = 3.0;   // if the flywheels never report full speed, feed anyway after this
    static final double FEED_TIME_S = 4.0;         // how long to feed once the gates open
    static final double DRIVE_TIMEOUT_S = 4.0;     // give up on a drive if it never settles

    // How far to drive left (toward -X, decreasing X) for the first move
    static final double LEAVE_LEFT_IN = 24;

    Drivebase drivebase;
    Intake intake;
    Shooter shooter;

    @Override
    public void runOpMode() throws InterruptedException {
        drivebase = new Drivebase(hardwareMap);
        intake = new Intake(hardwareMap);
        shooter = new Shooter(hardwareMap, drivebase, intake, gamepad1);
        shooter.setRumbleEnabled(false);   // no controller rumble during autonomous

        // Put the odometry in the shared field frame.
        drivebase.getPinpoint().resetPosAndIMU();
        sleep(300);   // let the Pinpoint finish its IMU reset before we write a pose to it
        Pose2D start = new Pose2D(DistanceUnit.INCH, 86, 132, AngleUnit.DEGREES, 270);
        drivebase.setCurrentPose(start);
        PoseStorage.currentPose = start;

        double startHeadingDeg = start.getHeading(AngleUnit.DEGREES);
        Location shootSpot = new Location(
                start.getX(FieldConstants.DISTANCE_UNIT),
                start.getY(FieldConstants.DISTANCE_UNIT),
                startHeadingDeg);
        // Leave by decreasing X while keeping the same Y and heading.
        Location moveLeft = new Location(
                105,
                130,
                startHeadingDeg);
        Location driveForwards = new Location(
                105,
                45,
                270);
        Location park = new Location(
                125,
                45,
                270);

        shooter.setTargetSpeeds(POLLEN_SPEED, NECTAR_SPEED);

        waitForStart();
        if (isStopRequested()) return;

        // Flywheels spin from the very start with the gates closed, and keep spinning for the whole auto.
        shooter.setState(Shooter.CaseModes.SHOOT_GATE_CLOSED);

        shoot();
        driveTo(moveLeft);
        driveTo(driveForwards);
        driveTo(park);

        // Nothing here ends the auto: keep the flywheels spinning and hold still until the driver station's
        // 30 second timer stops the opmode.
        while (opModeIsActive()) {
            drivebase.update();
            drivebase.drive(0, 0, 0);
            updateSubsystems();
            PoseStorage.currentPose = drivebase.getPosition();
            telemetry.addData("Step", "Done, holding");
            telemetry.update();
        }
        PoseStorage.currentPose = drivebase.getPosition();
    }

    /**
     * Opens the gates and feeds once both flywheels are at speed. If they haven't reached speed after
     * SPIN_UP_TIMEOUT_S it feeds anyway. Afterward the gates close again and the flywheels keep spinning.
     */
    void shoot() {
        shooter.setState(Shooter.CaseModes.SHOOT);
        ElapsedTime spinUpTimer = new ElapsedTime();
        ElapsedTime feedTimer = new ElapsedTime();
        boolean feeding = false;
        while (opModeIsActive()) {
            drivebase.update();
            boolean atSpeed = shooter.getPollenShooterVelocity() >= POLLEN_SPEED
                    && shooter.getNectarShooterVelocity() >= NECTAR_SPEED;
            if (!feeding && (atSpeed || spinUpTimer.seconds() >= SPIN_UP_TIMEOUT_S)) {
                feeding = true;
                feedTimer.reset();
            }
            if (feeding && feedTimer.seconds() >= FEED_TIME_S) break;

            shooter.setManualGateOpen(feeding);   // forces the gates open if we timed out waiting for speed
            shooter.update();
            if (feeding) intake.setState(Intake.CaseModes.ON);   // SHOOT only runs the intake at half speed until it's at speed
            intake.update();
            telemetry.addData("Step", feeding ? "Feeding" : "Spinning up");
            telemetry.addData("Pollen Speed", shooter.getPollenShooterVelocity());
            telemetry.addData("Nectar Speed", shooter.getNectarShooterVelocity());
            telemetry.update();
        }
        shooter.setManualGateOpen(false);
        shooter.setState(Shooter.CaseModes.SHOOT_GATE_CLOSED);
        intake.setState(Intake.CaseModes.OFF);
        updateSubsystems();
    }

    /** Keeps the shooter (flywheels and gates) and intake running; call once per loop pass. */
    void updateSubsystems() {
        shooter.update();
        intake.update();
    }

    /** Drives to a field target with Drivebase.driveToPosition until it arrives or times out. */
    void driveTo(Location target) {
        ElapsedTime timer = new ElapsedTime();
        while (opModeIsActive() && timer.seconds() < DRIVE_TIMEOUT_S) {
            drivebase.update();
            updateSubsystems();
            if (drivebase.isAtPosition(target, 1, 2)) break;
            drivebase.driveToPosition(target, telemetry);
            telemetry.addData("Step", "Driving");
            addPoseTelemetry();
        }
        drivebase.drive(0, 0, 0);
    }

    /** Holds the robot on a spot, cutting power once it is close enough so it doesn't jitter. */
    void holdOrDrive(Location target) {
        if (drivebase.isAtPosition(target, 1, 2)) {
            drivebase.drive(0, 0, 0);
        } else {
            drivebase.driveToPosition(target, telemetry);
        }
    }

    void addPoseTelemetry() {
        PoseStorage.currentPose = drivebase.getPosition();
        telemetry.addData("Position", drivebase.getPositionTelemetry());
        telemetry.update();
    }
}
