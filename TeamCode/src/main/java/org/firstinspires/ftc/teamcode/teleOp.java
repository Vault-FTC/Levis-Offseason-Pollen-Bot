package org.firstinspires.ftc.teamcode;

import com.qualcomm.hardware.gobilda.GoBildaPinpointDriver;
import com.qualcomm.hardware.limelightvision.LLResult;
import com.qualcomm.hardware.limelightvision.Limelight3A;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.PIDFCoefficients;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.Range;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.teamcode.prism.GoBildaPrismDriver;

@TeleOp
public class teleOp extends LinearOpMode {
    DcMotorEx shooter;
    @Override
    public void runOpMode() throws InterruptedException {
        DcMotor frontLeftMotor = hardwareMap.dcMotor.get("frontLeftMotor");
        DcMotor backLeftMotor = hardwareMap.dcMotor.get("backLeftMotor");
        DcMotor frontRightMotor = hardwareMap.dcMotor.get("frontRightMotor");
        DcMotor backRightMotor = hardwareMap.dcMotor.get("backRightMotor");
        DcMotorEx intake = hardwareMap.get(DcMotorEx.class, "intake");
        DcMotorEx transfer = hardwareMap.get(DcMotorEx.class, "transfer");
        // GoBildaPrismDriver
        GoBildaPinpointDriver pinpoint = hardwareMap.get(GoBildaPinpointDriver.class, "pinpoint");
        shooter = hardwareMap.get(DcMotorEx.class, "shooter");
        DcMotor lift = hardwareMap.dcMotor.get("lift");
        CRServo leftFlowerWheel = hardwareMap.get(CRServo.class, "leftFlowerWheel");
        CRServo rightFlowerWheel = hardwareMap.get(CRServo.class, "rightFlowerWheel");
        Servo flowerArm = hardwareMap.servo.get("flowerArm");
//        Servo bucketTilter = hardwareMap.servo.get("bucketTilter");
//        Servo servoGate = hardwareMap.servo.get("servoGate");
//        Servo SCA = hardwareMap.servo.get("SCA");
//        Limelight3A limelight3A = hardwareMap.get(Limelight3A.class, "limelight3A");

        double cameraHeightInches = 3.54;
        double targetHeightInches = 1.4;
        double cameraMountAngleDegrees = 0.0;
        double pollenShooterTargetSpeed = 1060;
        double nectarShooterTargetSpeed = 1150;
        double flowerArmStorePos = 0;
        double flowerArmPlacePos = 0.3;
        double flowerArmCollectPos = 0.37;
        double currentArmPosition = flowerArmStorePos;
        boolean liftAtUpPosition = false;
        boolean shooterReversed = false;
        int liftDownPosition = 0;
        int liftUpPosition = 1500;
        double liftManualPower = 0.5;
        boolean liftInManualMode = true;
//        int liftManualStepTicks = 2;
//        double position = lift.getCurrentPosition();
//        int liftUp = 1000;
//        int liftDown = 0;

        pinpoint.setOffsets(0.0, 145.0, DistanceUnit.MM);
        pinpoint.setEncoderResolution(GoBildaPinpointDriver.GoBildaOdometryPods.goBILDA_4_BAR_POD);
        pinpoint.setEncoderDirections(
                GoBildaPinpointDriver.EncoderDirection.FORWARD,
                GoBildaPinpointDriver.EncoderDirection.REVERSED);
        PIDFCoefficients pollenPIDF = new PIDFCoefficients(0.6, 0, 0, 14.2);
        PIDFCoefficients nectarPIDF = new PIDFCoefficients(0.6, 0, 0, 14.2);
//        limelight3A.pipelineSwitch(9);

        frontLeftMotor.setDirection(DcMotor.Direction.REVERSE);
        backLeftMotor.setDirection(DcMotor.Direction.REVERSE);
        intake.setDirection(DcMotorEx.Direction.REVERSE);
        transfer.setDirection(DcMotorEx.Direction.REVERSE);
        leftFlowerWheel.setDirection(CRServo.Direction.REVERSE);

        lift.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        lift.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        lift.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        shooter.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        shooter.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        shooter.setPIDFCoefficients(DcMotor.RunMode.RUN_USING_ENCODER, pollenPIDF);

        pinpoint.resetPosAndIMU();
        flowerArm.setPosition(flowerArmStorePos);
//        bucketTilter.setPosition(0.45);
//        SCA.setPosition(0.5);
//        limelight3A.start();

        waitForStart();

        if (isStopRequested()) return;

        while (opModeIsActive()) {
            pinpoint.update();
            double y, x, rx;

            if (gamepad1.optionsWasPressed()) {
                shooterReversed = !shooterReversed;
                shooter.setPIDFCoefficients(DcMotorEx.RunMode.RUN_USING_ENCODER,
                        shooterReversed ? nectarPIDF : pollenPIDF);
            }
            double currentTarget = shooterReversed ? -nectarShooterTargetSpeed : pollenShooterTargetSpeed;
            setShooterSpeed(currentTarget);

//          To get rid of the Limelight align, remove the whole "if (gamepad1.triangle)" block until "rx = gamepad1.right_stick_x;
//            }"
//          Then just replace it with this:
//               y = gamepad1.left_stick_y;
//                x = gamepad1.left_stick_x * 1.1;
//                rx = gamepad1.right_stick_x;

            y = gamepad1.left_stick_y;
            x = -gamepad1.left_stick_x * 1.1;
            rx = gamepad1.right_stick_x;
//            if (gamepad1.triangle) {
//                LLResult llResult = limelight3A.getLatestResult();
//
//                if (llResult != null && llResult.isValid()) {
//                    double tx = llResult.getTx();
//                    double ty = llResult.getTy();
//
//                    double kP_turn = -0.02; // TUNE THIS
//                    rx = Range.clip(-tx * kP_turn, -0.5, 0.5);
//
//                    double angleToTargetRadians = Math.toRadians(cameraMountAngleDegrees + ty);
//                    double distanceInches = (targetHeightInches - cameraHeightInches) / Math.tan(angleToTargetRadians);
//
//                    double kP_drive = 0.015; // TUNE THIS
//                    double distanceError = distanceInches - 10.0;
//                    y = (distanceError > 0) ? Range.clip(distanceError * kP_drive, 0, 0.5) : 0;
//
//                    x = 0; // no strafe correction
//
//                    telemetry.addData("tx", tx);
//                    telemetry.addData("distance (in)", distanceInches);
//                } else {
//                    y = 0; x = 0; rx = 0; // no valid target — hold still
//                }
//            } else {
//                y = -gamepad1.left_stick_y;
//                x = gamepad1.left_stick_x * 1.1;
//                rx = gamepad1.right_stick_x;
//            }
            double heading = pinpoint.getHeading(AngleUnit.RADIANS);
            double rotX = x * Math.cos(-heading) - y * Math.sin(-heading);
            double rotY = x * Math.sin(-heading) + y * Math.cos(-heading);

            double denominator = Math.max(Math.abs(rotY) + Math.abs(rotX) + Math.abs(rx), 1);
            double frontLeftPower = (rotY + rotX + rx) / denominator;
            double backLeftPower = (rotY - rotX + rx) / denominator;
            double frontRightPower = (rotY - rotX - rx) / denominator;
            double backRightPower = (rotY + rotX - rx) / denominator;

//            double denominator = Math.max(Math.abs(y) + Math.abs(x) + Math.abs(rx), 1);
//            double frontLeftPower = (y + x + rx) / denominator;
//            double backLeftPower = (y - x + rx) / denominator;
//            double frontRightPower = (y - x - rx) / denominator;
//            double backRightPower = (y + x - rx) / denominator;

//            double denominator = Math.max(Math.abs(y) + Math.abs(x) + Math.abs(rx), 1);
//            double frontLeftPower = (y + x + rx) / denominator;
//            double backLeftPower = (y - x + rx) / denominator;
//            double frontRightPower = (y + x - rx) / denominator;
//            double backRightPower = (y - x - rx) / denominator;


            doIntake(intake, transfer);

            if (gamepad1.left_stick_button) {
                leftFlowerWheel.setPower(1.0);
                rightFlowerWheel.setPower(1.0);
            }
            else if (gamepad1.right_stick_button) {
                leftFlowerWheel.setPower(-1.0);
                rightFlowerWheel.setPower(-1.0);
            }
            else {
                leftFlowerWheel.setPower(0.0);
                rightFlowerWheel.setPower(0.0);
            }

            if (gamepad1.dpadDownWasPressed()) {
                if (currentArmPosition == flowerArmStorePos) {
                    currentArmPosition = flowerArmPlacePos;
                } else if (currentArmPosition == flowerArmPlacePos) {
                    currentArmPosition = flowerArmCollectPos;
                }
                flowerArm.setPosition(currentArmPosition);
            }
            if (gamepad1.dpadUpWasPressed()) {
                if (currentArmPosition == flowerArmCollectPos) {
                    currentArmPosition = flowerArmPlacePos;
                }
                else if (currentArmPosition == flowerArmPlacePos) {
                    currentArmPosition = flowerArmStorePos;
                }
                flowerArm.setPosition(currentArmPosition);
            }

            if (gamepad1.triangleWasPressed()) {
                liftInManualMode = false;
                liftAtUpPosition = !liftAtUpPosition;
                lift.setTargetPosition(liftAtUpPosition ? liftUpPosition : liftDownPosition);
                lift.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                lift.setPower(1);
            }
            if (gamepad1.circle) {
                liftInManualMode = true;
                lift.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
                lift.setPower(liftManualPower);
//                int newTarget = Math.min(lift.getTargetPosition() + liftManualStepTicks, liftMaxPosition);
//                lift.setTargetPosition(newTarget);
//                lift.setMode(DcMotor.RunMode.RUN_TO_POSITION);
//                lift.setPower(1);
            }
            else if (gamepad1.cross) {
                liftInManualMode = true;
                lift.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
                lift.setPower(-liftManualPower);
//                int newTarget = Math.max(lift.getTargetPosition() - liftManualStepTicks, liftMinPosition);
//                lift.setTargetPosition(newTarget);
//                lift.setMode(DcMotor.RunMode.RUN_TO_POSITION);
//                lift.setPower(1);
            }
            else if (liftInManualMode) {
                lift.setPower(0);
            }
//            if (gamepad1.x) {
//                lift.setPower(0.5);
//            }
//            else if (gamepad1.a) {
//                lift.setPower(-0.5);
//            }
//            else lift.setPower(0);
//            if (gamepad1.yWasPressed()) {
//                bucketTilter.setPosition(0.4);
////                //servoGatepos = Range.clip(servoGatepos + 0.1, 0, 1);
//            }
//            if (gamepad1.bWasPressed()) {
//                bucketTilter.setPosition(0.5);
////                //servoGatepos = Range.clip(servoGatepos - 0.1, 0, 1);
//            }
//            if (gamepad1.startWasPressed()) {
//                bucketTilter.setPosition(0.45);
////                //servoposition = Range.clip(servoposition + 0.1, 0, 1);
//            }
//            else if (gamepad1.backWasPressed()) {
//                //servoposition = Range.clip(servoposition - 0.1, 0, 1);
//            }
            //bucketTilter.setPosition(servoposition);
            //servoGate.setPosition(servoGatepos);
//            if (gamepad1.x) {
//                lift.setTargetPosition(liftUp);
//            }
//            if (gamepad1.y) {
//                lift.setTargetPosition(liftDown);
//            }

            frontLeftMotor.setPower(frontLeftPower);
            backLeftMotor.setPower(backLeftPower);
            frontRightMotor.setPower(frontRightPower);
            backRightMotor.setPower(backRightPower);

            boolean shooterReady = Math.abs(shooter.getVelocity() - currentTarget) < 100;

            telemetry.addData("Shooter Mode", shooterReversed ? "Nectar" : "Pollen");
            telemetry.addData("Shooter Ready", shooterReady);
            telemetry.addData("Shooter Target", currentTarget);
            telemetry.addData("Shooter Speed", shooter.getVelocity());
            telemetry.addData("Pos X (mm)", pinpoint.getPosX(DistanceUnit.MM));
            telemetry.addData("Pos Y (mm)", pinpoint.getPosY(DistanceUnit.MM));
            telemetry.addData("Heading (deg)", pinpoint.getHeading(AngleUnit.DEGREES));
            telemetry.addData("Shooter Reversed (raw)", shooterReversed);
            telemetry.addData("Arm Position", currentArmPosition);
            telemetry.addData("Lift Target", lift.getTargetPosition());
            telemetry.addData("Lift Current", lift.getCurrentPosition());
            telemetry.addData("Shooter Position (ticks)", shooter.getCurrentPosition());
            telemetry.addData("Flower Arm Position", flowerArm.getPosition());
//            telemetry.addData("bucketTilter actual", bucketTilter.getPosition());
            telemetry.update();
        }
    }

    public void doIntake(DcMotorEx intake, DcMotorEx transfer) {
        if (gamepad1.right_bumper) {
            intake.setPower(1);
            transfer.setPower(1);
        }
        else if (gamepad1.left_bumper){
            intake.setPower(1);
            transfer.setPower(0);
        }
        else if (gamepad1.left_trigger_pressed) {
            transfer.setPower(1);
            intake.setPower(0);
        }
        else if (gamepad1.square) {
            transfer.setPower(-1);
            intake.setPower(-1);
        }
        else {
            intake.setPower(0);
            transfer.setPower(0);
        }
    }

    public void setShooterSpeed(double speed){
        shooter.setVelocity(speed);
    }
}