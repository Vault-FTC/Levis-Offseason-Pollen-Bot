package org.firstinspires.ftc.teamcode.autonomi.redAutons;

import com.qualcomm.hardware.gobilda.GoBildaPinpointDriver;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.robotcore.external.navigation.Pose2D;
import org.firstinspires.ftc.teamcode.prism.GoBildaPrismDriver;

@Autonomous
public class redAuto extends LinearOpMode {
    DcMotorEx shooter;
    double cellPosition; // this tells us what position the hive is in. If 1, the upward cell is on the right when standing in the blue driver station. if zero, that side is down.
    Pose2D BlueGoalCaseOne = new Pose2D(DistanceUnit.CM, 105, 83, AngleUnit.RADIANS, 0);
    Pose2D BlueGoalCaseTwo = new Pose2D(DistanceUnit.CM, 105, 60, AngleUnit.RADIANS, 0);
    GoBildaPinpointDriver pinpoint;

    @Override
    public void runOpMode() throws InterruptedException {
        DcMotor frontLeftMotor = hardwareMap.dcMotor.get("frontLeftMotor");
        DcMotor backLeftMotor = hardwareMap.dcMotor.get("backLeftMotor");
        DcMotor frontRightMotor = hardwareMap.dcMotor.get("frontRightMotor");
        DcMotor backRightMotor = hardwareMap.dcMotor.get("backRightMotor");
        DcMotorEx intake = hardwareMap.get(DcMotorEx.class, "intake");
        DcMotorEx transfer = hardwareMap.get(DcMotorEx.class, "transfer");
        GoBildaPrismDriver prism = hardwareMap.get(GoBildaPrismDriver.class, "prism");
        pinpoint = hardwareMap.get(GoBildaPinpointDriver.class, "pinpoint");
        shooter = hardwareMap.get(DcMotorEx.class, "shooter");
        DcMotor lift = hardwareMap.dcMotor.get("lift");

        double pollenShooterTargetSpeed = 950;
        double nectarShooterTargetSpeed = 1125;

        frontLeftMotor.setDirection(DcMotor.Direction.REVERSE);
        backLeftMotor.setDirection(DcMotor.Direction.REVERSE);
        intake.setDirection(DcMotorEx.Direction.REVERSE);
        transfer.setDirection(DcMotorEx.Direction.REVERSE);

        waitForStart();

        if (isStopRequested()) return;
        shooter.setVelocity(nectarShooterTargetSpeed);
        wait(2);
        intake.setPower(1);
        transfer.setPower(0.3);
        wait(4);
        frontLeftMotor.setPower(-1);
        frontRightMotor.setPower(1);
        backLeftMotor.setPower(1);
        backRightMotor.setPower(-1);
        wait(3);
        frontLeftMotor.setPower(0);
        frontRightMotor.setPower(0);
        backLeftMotor.setPower(0);
        backRightMotor.setPower(0);
    }

}
