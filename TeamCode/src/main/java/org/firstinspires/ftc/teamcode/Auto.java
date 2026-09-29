package org.firstinspires.ftc.teamcode;

import com.qualcomm.hardware.gobilda.GoBildaPinpointDriver;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.PIDFCoefficients;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;

@Autonomous
public class Auto extends LinearOpMode {

    DcMotor frontLeftMotor = hardwareMap.dcMotor.get("frontLeftMotor");
    DcMotor backLeftMotor = hardwareMap.dcMotor.get("backLeftMotor");
    DcMotor frontRightMotor = hardwareMap.dcMotor.get("frontRightMotor");
    DcMotor backRightMotor = hardwareMap.dcMotor.get("backRightMotor");
    DcMotorEx intake = hardwareMap.get(DcMotorEx.class, "intake");
    DcMotorEx transfer = hardwareMap.get(DcMotorEx.class, "transfer");
    DcMotorEx shooter = hardwareMap.get(DcMotorEx.class, "shooter");
    PIDFCoefficients pollenPIDF = new PIDFCoefficients(0.6, 0, 0, 14.2);
    GoBildaPinpointDriver odo;
    @Override
    public void runOpMode() throws InterruptedException {

        shooter.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        shooter.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        shooter.setPIDFCoefficients(DcMotor.RunMode.RUN_USING_ENCODER, pollenPIDF);
        frontLeftMotor.setDirection(DcMotor.Direction.REVERSE);
        backLeftMotor.setDirection(DcMotor.Direction.REVERSE);
        intake.setDirection(DcMotorEx.Direction.REVERSE);
        transfer.setDirection(DcMotorEx.Direction.REVERSE);
        odo = hardwareMap.get(GoBildaPinpointDriver.class,"odo");
        waitForStart();

    }


    public void driveToPosition(Location target, double turnVal, Telemetry telemetry) {
        double p = 0.2; //0.1
        double p_rotation = 0.015;
        double strafe = (target.Strafe - odo.getPosY(DistanceUnit.CM));
        double forward = (-target.Forward + odo.getPosX(DistanceUnit.CM));
        double heading = (target.TurnDegrees - odo.getHeading(AngleUnit.DEGREES));

        double strafeError = (target.Strafe - odo.getPosY(DistanceUnit.CM));
        double forwardError = (-target.Forward + odo.getPosX(DistanceUnit.CM));
        double headingError = (target.TurnDegrees - odo.getHeading(AngleUnit.DEGREES));


        if (telemetry != null) {
            telemetry.addData("Target", "X: " + target.Strafe + "  Y: " + target.Forward);
        }

        double forwardPower = forward * p;
        double strafePower = strafe * p;
        double turnPower = heading * p_rotation;

        double minPower = 0.25;
        double maxPower = 0.4;

        forwardPower = Math.max(Math.min(forwardPower, maxPower), -maxPower);
        strafePower = Math.max(Math.min(strafePower, maxPower), -maxPower);

        if (Math.abs(forwardPower) < minPower && Math.abs(forwardError) > 3) {
            forwardPower = minPower * Math.signum(forwardError);
        }
        if (Math.abs(strafePower) < minPower && Math.abs(strafeError) > 3) {
            strafePower = minPower * Math.signum(strafeError);
        }

        drive(forwardPower, strafePower, turnPower);

    }

    public void drive(double forward, double right, double rotate) {
        double botHeading = -odo.getHeading(AngleUnit.RADIANS);
//         X is positive up, Y is positive to the right
        double rotRight = right * Math.cos(botHeading) - forward * Math.sin(botHeading);
        double rotForward = right * Math.sin(botHeading) + forward * Math.cos(botHeading);

        double frontLeftPower = rotForward + rotRight + rotate;
        double backLeftPower = rotForward - rotRight + rotate;
        double frontRightPower = rotForward - rotRight - rotate;
        double backRightPower = rotForward + rotRight - rotate;

        // Calculate motor powers

        double maxPower = Math.max(Math.abs(frontLeftPower),
                Math.max(Math.abs(backLeftPower),
                        Math.max(Math.abs(frontRightPower), Math.abs(backRightPower))));

        double SpeedLimit = 1;

        if (maxPower > 1.0) {
            frontLeftPower /= maxPower;
            backLeftPower /= maxPower;
            frontRightPower /= maxPower;
            backRightPower /= maxPower;
        }
        frontLeftPower *= SpeedLimit;
        backLeftPower *= SpeedLimit;
        frontRightPower *= SpeedLimit;
        backRightPower *= SpeedLimit;

        frontLeftPower = Math.abs(frontLeftPower) < 0.02 ? 0 : frontLeftPower;
        frontRightPower = Math.abs(frontRightPower) < 0.02 ? 0 : frontRightPower;
        backLeftPower = Math.abs(backLeftPower) < 0.02 ? 0 : backLeftPower;
        backRightPower = Math.abs(backRightPower) < 0.02 ? 0 : backRightPower;

        // Set motor powers
        frontLeftMotor.setPower(frontLeftPower);
        backLeftMotor.setPower(backLeftPower);
        frontRightMotor.setPower(frontRightPower);
        backRightMotor.setPower(backRightPower);
    }
}
