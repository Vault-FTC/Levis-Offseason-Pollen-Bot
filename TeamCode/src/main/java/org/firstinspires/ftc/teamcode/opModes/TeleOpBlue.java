package org.firstinspires.ftc.teamcode.opModes;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.robotcore.external.navigation.Pose2D;
import org.firstinspires.ftc.teamcode.subsystems.Drivebase;
import org.firstinspires.ftc.teamcode.subsystems.Intake;
import org.firstinspires.ftc.teamcode.subsystems.Shooter;

@com.qualcomm.robotcore.eventloop.opmode.TeleOp
public class TeleOpBlue extends LinearOpMode {
    Drivebase drivebase;
    Intake intake;
    Shooter shooter;
    double errorDeg;
    double rx;
    double cellPosition; // this tells us what position the hive is in. If 1, the upward cell is on the right when standing in the blue driver station. if zero, that side is down.
    Pose2D BlueGoalCaseOne = new Pose2D(DistanceUnit.CM, 105, 83, AngleUnit.RADIANS, 0);
    Pose2D BlueGoalCaseTwo = new Pose2D(DistanceUnit.CM, 105, 60, AngleUnit.RADIANS, 0);

    @Override
    public void runOpMode() throws InterruptedException {
        drivebase = new Drivebase(hardwareMap);
        intake = new Intake(hardwareMap);
        shooter = new Shooter(hardwareMap, drivebase, intake, gamepad1);

        double pollenTargetSpeed = 950;
        double nectarTargetSpeed = 1125;
        double pollenSpeedAdjustment = 0;
        double nectarSpeedAdjustment = 0;
        boolean pollenSelected = true;   // which shooter gamepad2's d-pad adjusts
        cellPosition = 1; //cell starts with blue alliance cell up, red alliance cell down.

        drivebase.getPinpoint().resetPosAndIMU();
        drivebase.setCurrentPose(0, 0, 0);

        waitForStart();

        if (isStopRequested()) return;

        while (opModeIsActive()) {
            drivebase.update();

            double y = gamepad1.left_stick_y;
            double x = -gamepad1.left_stick_x * 1.1;
            rx = gamepad1.right_stick_x;

            // DRIVER ONE
            doIntake();

            // Shooters are always spinning with the gates closed. Right bumper: auto-aim, then open the
            // gates and feed once both flywheels are at speed. Right trigger: same shot, no auto-aim.
            if (gamepad1.right_bumper) {
                autoAim();   // adds a correction to rx
                shooter.setState(Shooter.CaseModes.SHOOT);
            } else if (gamepad1.right_trigger_pressed) {
                shooter.setState(Shooter.CaseModes.SHOOT);
            } else {
                shooter.setState(Shooter.CaseModes.SHOOT_GATE_CLOSED);
            }

            drivebase.drive(y, x, rx);

            // DRIVER TWO
            if (gamepad2.squareWasPressed()) {
                cellPosition = (cellPosition == 1) ? 0 : 1;
            }
            if (gamepad2.circleWasPressed()) {
                pollenSelected = !pollenSelected;
            }
            if (gamepad2.dpadUpWasPressed()) {
                if (pollenSelected) pollenSpeedAdjustment += 25;
                else nectarSpeedAdjustment += 25;
            }
            if (gamepad2.dpadDownWasPressed()) {
                if (pollenSelected) pollenSpeedAdjustment -= 25;
                else nectarSpeedAdjustment -= 25;
            }
            if (gamepad2.right_bumper) {
                double positionInCycle = System.currentTimeMillis() % 1000;
                intake.setState(positionInCycle < 100 ? Intake.CaseModes.ON : Intake.CaseModes.OFF);
            }
            shooter.setManualGateOpen(gamepad1.triangle || gamepad2.triangle);   // hold Triangle to force the gates open

            shooter.setTargetSpeeds(pollenTargetSpeed + pollenSpeedAdjustment, nectarTargetSpeed + nectarSpeedAdjustment);
            shooter.update();   // in SHOOT mode this overrides the intake state
            intake.update();

            telemetry.addData("Selected Shooter (gamepad2 Circle)", pollenSelected ? "Pollen" : "Nectar");
            telemetry.addData("Shooter Target (pollen)", pollenTargetSpeed + pollenSpeedAdjustment);
            telemetry.addData("Shooter Target (nectar)", nectarTargetSpeed + nectarSpeedAdjustment);
            telemetry.addData("Pollen Speed", shooter.getPollenShooterVelocity());
            telemetry.addData("Nectar Speed", shooter.getNectarShooterVelocity());
            telemetry.addData("Position", drivebase.getPositionTelemetry());
            telemetry.addData("Cell Position", cellPosition);
            telemetry.addData("Angle to goal", errorDeg = (Drivebase.angleToGoal(drivebase.getPosition(), BlueGoalCaseOne)) * (180 / Math.PI));
            telemetry.update();
        }
    }

    public void doIntake() {
        if (gamepad1.left_bumper) {
            intake.setState(Intake.CaseModes.ON);
        } else if (gamepad1.square) {
            intake.setState(Intake.CaseModes.REVERSE);
        } else {
            intake.setState(Intake.CaseModes.OFF);
        }
    }

    public void autoAim() {
        Pose2D goal = (cellPosition == 1) ? BlueGoalCaseOne : BlueGoalCaseTwo;
        errorDeg = (Drivebase.angleToGoal(drivebase.getPosition(), goal)) * (180 / Math.PI);
        rx = rx + errorDeg * -0.015;
    }
}
