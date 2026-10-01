package org.firstinspires.ftc.teamcode.opModes;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.PIDFCoefficients;

/**
 * Tunes the pollen and nectar shooters at the same time. Both motors always run at their own target velocity;
 * Options selects which one the Cross/Square and D-Pad adjustments apply to.
 */
@TeleOp
public class ShooterPIDTuner extends OpMode {
    /** Tuning state for one shooter motor. */
    private static class Flywheel {
        final String name;
        final DcMotorEx motor;
        double targetVelocity = 0;
        double P = 0;
        double F = 14.5;

        Flywheel(String name, DcMotorEx motor) {
            this.name = name;
            this.motor = motor;
        }

        void apply() {
            motor.setPIDFCoefficients(DcMotor.RunMode.RUN_USING_ENCODER, new PIDFCoefficients(P, 0, 0, F));
            motor.setVelocity(targetVelocity);
        }
    }

    public DcMotorEx intake;
    Flywheel pollen;
    Flywheel nectar;
    Flywheel selected;

    double[] pidStepSizes = {10.0, 1.0, 0.1, 0.01, 0.001, 0.0001};
    int pidStepIndex = 0;

    double[] velocityStepSizes = {10.0, 50.0, 100.0, 500.0, 950, 1000.0};
    int velocityStepIndex = 0;

    @Override
    public void init() {
        pollen = new Flywheel("Pollen", hardwareMap.get(DcMotorEx.class, "pollenShooter"));
        nectar = new Flywheel("Nectar", hardwareMap.get(DcMotorEx.class, "nectarShooter"));
        intake = hardwareMap.get(DcMotorEx.class, "intake");
        selected = pollen;

        for (Flywheel f : new Flywheel[]{pollen, nectar}) {
            f.motor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            f.motor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            f.apply();
        }
        intake.setDirection(DcMotorEx.Direction.REVERSE);
        telemetry.addLine("Init Complete");
    }

    @Override
    public void loop() {
        // Options: choose which shooter the adjustments below apply to
        if (gamepad1.optionsWasPressed()) {
            selected = (selected == pollen) ? nectar : pollen;
        }

        // Cycle which velocity step size is active
        if (gamepad1.triangleWasPressed()) {
            velocityStepIndex = (velocityStepIndex + 1) % velocityStepSizes.length;
        }
        // Adjust the selected shooter's target velocity by the active step size
        if (gamepad1.crossWasPressed()) {
            selected.targetVelocity += velocityStepSizes[velocityStepIndex];
        }
        if (gamepad1.squareWasPressed()) {
            selected.targetVelocity -= velocityStepSizes[velocityStepIndex];
        }

        // Cycle which P/F step size is active
        if (gamepad1.circleWasPressed()) {
            pidStepIndex = (pidStepIndex + 1) % pidStepSizes.length;
        }
        if (gamepad1.dpadRightWasPressed()) {
            selected.F += pidStepSizes[pidStepIndex];
        }
        if (gamepad1.dpadLeftWasPressed()) {
            selected.F -= pidStepSizes[pidStepIndex];
        }
        if (gamepad1.dpadDownWasPressed()) {
            selected.P -= pidStepSizes[pidStepIndex];
        }
        if (gamepad1.dpadUpWasPressed()) {
            selected.P += pidStepSizes[pidStepIndex];
        }

        // Feed a game piece so you're tuning under realistic load instead of spinning free.
        intake.setPower(gamepad1.left_bumper ? 1 : 0);

        pollen.apply();
        nectar.apply();

        telemetry.addData("Selected (Options to switch)", selected.name);
        telemetry.addData("Velocity Step Size", "%.0f (Triangle to cycle, Cross +/Square -)", velocityStepSizes[velocityStepIndex]);
        telemetry.addData("PID Step Size", "%.4f (Circle button)", pidStepSizes[pidStepIndex]);
        for (Flywheel f : new Flywheel[]{pollen, nectar}) {
            double curVelocity = f.motor.getVelocity();
            telemetry.addLine("----------- " + f.name + " -----------");
            telemetry.addData(f.name + " Target Velocity", "%.0f", f.targetVelocity);
            telemetry.addData(f.name + " Current Velocity", "%.2f", curVelocity);
            telemetry.addData(f.name + " Error", "%.2f", f.targetVelocity - curVelocity);
            telemetry.addData(f.name + " P (D-Pad U/D)", "%.4f", f.P);
            telemetry.addData(f.name + " F (D-Pad L/R)", "%.4f", f.F);
        }
    }
}
