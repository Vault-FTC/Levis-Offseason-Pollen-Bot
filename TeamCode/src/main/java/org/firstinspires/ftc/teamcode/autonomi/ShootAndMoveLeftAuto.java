package org.firstinspires.ftc.teamcode.autonomi;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.teamcode.subsystems.Drivebase;
import org.firstinspires.ftc.teamcode.subsystems.Intake;
import org.firstinspires.ftc.teamcode.subsystems.Shooter;

/**
 * Basic auto that doesn't need Pedro: shoot, strafe left 2 ft, drive forward 8 ft, strafe left 2 ft.
 * Each move is measured with the Pinpoint.
 */
@Autonomous(name = "Shoot And Move Left")
public class ShootAndMoveLeftAuto extends LinearOpMode {
    static final double POLLEN_SPEED = 1300;
    static final double NECTAR_SPEED = 1300;
    static final double SHOOT_SECONDS = 6.0;        // spin-up + feeding time

    static final double CM_PER_FOOT = 30.48;
    static final double MOVE_POWER = 0.4;
    static final double SLOW_MOVE_POWER = 0.25;     // used for the last SLOW_ZONE_CM of each move
    static final double SLOW_ZONE_CM = 30;
    static final double SETTLE_MS = 250;            // pause between moves so the robot stops before the next one

    // Drivebase.drive(forward, right, rotate): a positive "right" value strafes the robot LEFT (same as the
    // teleop's x = -left_stick_x) and a NEGATIVE "forward" value drives the robot forward (same as the teleop's
    // y = left_stick_y). If the robot goes the wrong way, flip the matching sign to -1.
    static final double LEFT_SIGN = 1;
    static final double FORWARD_SIGN = -1;

    Drivebase drivebase;

    @Override
    public void runOpMode() throws InterruptedException {
        drivebase = new Drivebase(hardwareMap);
        Intake intake = new Intake(hardwareMap);
        Shooter shooter = new Shooter(hardwareMap, drivebase, intake, gamepad1);
        shooter.setRumbleEnabled(false);   // no controller rumble during autonomous

        drivebase.getPinpoint().resetPosAndIMU();
        drivebase.setCurrentPose(0, 0, Math.PI);   // the robot starts turned 180 degrees, same as teleop
        shooter.setTargetSpeeds(POLLEN_SPEED, NECTAR_SPEED);

        waitForStart();
        if (isStopRequested()) return;

        // 1. Shoot: SHOOT spins both flywheels up, then opens the gates and runs the intake at speed.
        shooter.setState(Shooter.CaseModes.SHOOT);
        ElapsedTime timer = new ElapsedTime();
        while (opModeIsActive() && timer.seconds() < SHOOT_SECONDS) {
            drivebase.update();
            shooter.update();
            intake.update();
            telemetry.addData("Step", "Shooting");
            telemetry.addData("Pollen Speed", shooter.getPollenShooterVelocity());
            telemetry.addData("Nectar Speed", shooter.getNectarShooterVelocity());
            telemetry.update();
        }
        shooter.setState(Shooter.CaseModes.OFF);   // stops the flywheels and closes the gates
        intake.setState(Intake.CaseModes.OFF);
        shooter.update();
        intake.update();

        // 2. Left 2 ft, forward 8 ft, left 2 ft.
        move("Left 2 ft", 0, -1, 2 * CM_PER_FOOT, 4);
        move("Forward 8 ft", -1, 0, 8 * CM_PER_FOOT, 8);
        move("Left 2 ft", 0, 1, 2 * CM_PER_FOOT, 4);

        telemetry.addData("Done", "");
        telemetry.update();
    }

    /**
     * Drives until the Pinpoint says the robot has travelled distanceCm from where the move started.
     * forwardDir and leftDir are 1 for "go that way" and 0 for "don't".
     */
    private void move(String label, double forwardDir, double leftDir, double distanceCm, double timeoutSeconds) {
        if (!opModeIsActive()) return;
        drivebase.update();
        double startX = drivebase.getPinpoint().getPosX(DistanceUnit.CM);
        double startY = drivebase.getPinpoint().getPosY(DistanceUnit.CM);
        ElapsedTime timer = new ElapsedTime();
        double travelled = 0;
        while (opModeIsActive() && timer.seconds() < timeoutSeconds) {
            drivebase.update();
            travelled = Math.hypot(
                    drivebase.getPinpoint().getPosX(DistanceUnit.CM) - startX,
                    drivebase.getPinpoint().getPosY(DistanceUnit.CM) - startY);
            if (travelled >= distanceCm) break;

            double power = (distanceCm - travelled < SLOW_ZONE_CM) ? SLOW_MOVE_POWER : MOVE_POWER;
            drivebase.drive(FORWARD_SIGN * forwardDir * power, LEFT_SIGN * leftDir * power, 0);
            telemetry.addData("Step", label);
            telemetry.addData("Travelled (cm)", travelled);
            telemetry.addData("Target (cm)", distanceCm);
            telemetry.update();
        }
        drivebase.drive(0, 0, 0);
        sleep((long) SETTLE_MS);
    }
}
