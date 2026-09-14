package org.firstinspires.ftc.teamcode;

import com.qualcomm.hardware.gobilda.GoBildaPinpointDriver;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.PIDFCoefficients;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;

@TeleOp
public class ShooterPIDTuner extends OpMode {
    public DcMotorEx shooter;
    public DcMotorEx intake;
    public DcMotorEx transfer;
    public GoBildaPinpointDriver pinpoint;

    double curTargetVelocity = 0;
    double F = 14.2;
    double P = 0;

    double[] pidStepSizes = {10.0, 1.0, 0.1, 0.001, 0.0001};
    int pidStepIndex = 0;

    double[] velocityStepSizes = {10.0, 50.0, 100.0, 500.0, 1000.0, 1100.0};
    int velocityStepIndex = 0;

    @Override
    public void init() {
        shooter = hardwareMap.get(DcMotorEx.class, "shooter");
        intake = hardwareMap.get(DcMotorEx.class, "intake");
        transfer = hardwareMap.get(DcMotorEx.class, "transfer");
        pinpoint = hardwareMap.get(GoBildaPinpointDriver.class, "pinpoint");

        shooter.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        shooter.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        PIDFCoefficients pidfCoefficients = new PIDFCoefficients(P, 0, 0, F);
        shooter.setPIDFCoefficients(DcMotor.RunMode.RUN_USING_ENCODER, pidfCoefficients);
        telemetry.addLine("Init Complete");

        intake.setDirection(DcMotorEx.Direction.REVERSE);
        transfer.setDirection(DcMotorSimple.Direction.REVERSE);

        pinpoint.setOffsets(0.0, 145.0, DistanceUnit.MM);
        pinpoint.setEncoderResolution(GoBildaPinpointDriver.GoBildaOdometryPods.goBILDA_4_BAR_POD);
        pinpoint.setEncoderDirections(
                GoBildaPinpointDriver.EncoderDirection.FORWARD,
                GoBildaPinpointDriver.EncoderDirection.REVERSED);
    }

    @Override
    public void loop() {
        pinpoint.update();

        // Cycle which velocity step size is active
        if (gamepad1.triangleWasPressed()) {
            velocityStepIndex = (velocityStepIndex + 1) % velocityStepSizes.length;
        }
        // Adjust target velocity by the active step size
        if (gamepad1.crossWasPressed()) {
            curTargetVelocity += velocityStepSizes[velocityStepIndex];
        }
        if (gamepad1.squareWasPressed()) {
            curTargetVelocity -= velocityStepSizes[velocityStepIndex];
        }

        // Cycle which P/F step size is active
        if (gamepad1.circleWasPressed()) {
            pidStepIndex = (pidStepIndex + 1) % pidStepSizes.length;
        }
        if (gamepad1.dpadRightWasPressed()) {
            F += pidStepSizes[pidStepIndex];
        }
        if (gamepad1.dpadLeftWasPressed()) {
            F -= pidStepSizes[pidStepIndex];
        }
        if (gamepad1.dpadDownWasPressed()) {
            P -= pidStepSizes[pidStepIndex];
        }
        if (gamepad1.dpadUpWasPressed()) {
            P += pidStepSizes[pidStepIndex];
        }

        // Load the shooter with a game piece the same way your real OpMode does,
        // so you're tuning under realistic load instead of tuning it spinning free.
        if (gamepad1.left_bumper) {
            intake.setPower(1);
            transfer.setPower(1);
        } else {
            intake.setPower(0);
            transfer.setPower(0);
        }

        PIDFCoefficients pidfCoefficients = new PIDFCoefficients(P, 0, 0, F);
        shooter.setPIDFCoefficients(DcMotor.RunMode.RUN_USING_ENCODER, pidfCoefficients);
        shooter.setVelocity(curTargetVelocity);
        double curVelocity = shooter.getVelocity();
        double error = curTargetVelocity - curVelocity;

        telemetry.addData("Target Velocity", curTargetVelocity);
        telemetry.addData("Velocity Step Size", "%.0f (Triangle to cycle, Cross +/Square -)", velocityStepSizes[velocityStepIndex]);
        telemetry.addData("Current Velocity", "%.2f", curVelocity);
        telemetry.addData("Error", "%.2f", error);
        telemetry.addLine("-----------------------");
        telemetry.addData("Tuning P", "%.4f (D-Pad U/D)", P);
        telemetry.addData("Tuning F", "%.4f (D-Pad L/R)", F);
        telemetry.addData("PID Step Size", "%.4f (Circle button)", pidStepSizes[pidStepIndex]);
        telemetry.addData("Pos X (mm)", pinpoint.getPosX(DistanceUnit.MM));
        telemetry.addData("Pos Y (mm)", pinpoint.getPosY(DistanceUnit.MM));
        telemetry.addData("Heading (deg)", pinpoint.getHeading(AngleUnit.DEGREES));
    }
}