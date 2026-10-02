package org.firstinspires.ftc.teamcode.subsystems;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.PIDFCoefficients;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.robotcore.external.navigation.UnnormalizedAngleUnit;
import org.firstinspires.ftc.teamcode.commandSystem.Subsystem;

public class Shooter extends Subsystem {
    public enum CaseModes {
        OFF, SHOOT, SHOOT_GATE_CLOSED, REVERSE
    }

    private final DcMotorEx pollenShooter;
    private final DcMotorEx nectarShooter;
    private final Servo pollenGate;
    private final Servo nectarGate;
    private final Drivebase drivebase;
    private final Intake intake;
    double pollenSpeed;
    double nectarSpeed;
    double kP = 0.02; //check this
    double kD = 0.0015; //check this
    CaseModes currentMode = CaseModes.OFF;
    boolean manualGateOpen = false;
    boolean shootReady = false;   // latched once the flywheels reach speed in SHOOT; cleared when leaving SHOOT
    PIDFCoefficients pollenPIDF = new PIDFCoefficients(160, 0, 0, 15.5);
    PIDFCoefficients nectarPIDF = new PIDFCoefficients(300, 0, 0, 14.996);
    Gamepad gamepad1; //what is this?

    public Shooter(HardwareMap hardwareMap, Drivebase driveBase, Intake intake, Gamepad gamepad) {
        pollenShooter = hardwareMap.get(DcMotorEx.class, "pollenShooter");
        nectarShooter = hardwareMap.get(DcMotorEx.class, "nectarShooter");
        pollenGate = hardwareMap.get(Servo.class, "pollenGate");
        nectarGate = hardwareMap.get(Servo.class, "nectarGate");
        this.drivebase = driveBase;
        this.intake = intake;
        pollenShooter.setDirection(DcMotorEx.Direction.REVERSE);
        pollenShooter.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        pollenShooter.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        pollenShooter.setPIDFCoefficients(DcMotor.RunMode.RUN_USING_ENCODER, pollenPIDF);
        nectarShooter.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        nectarShooter.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        nectarShooter.setPIDFCoefficients(DcMotor.RunMode.RUN_USING_ENCODER, nectarPIDF);
        gamepad1 = gamepad; //what is this?
    }

    public void update() {
//        double angleError = Drivebase.angleToGoal(drivebase.getPosition(), goal);
        double velocityDeg = drivebase.getPinpoint().getHeadingVelocity(UnnormalizedAngleUnit.DEGREES);
        //drivebase.updateAutoAim(0);
//        double offset_by_distance = 0.0;
//        distance = drivebase.distanceToGoal(drivebase.getPosition(), goal);
//        setShooterSpeed(distanceToSpeed(distance));
        boolean gateOpen = false;
        switch(currentMode){
            case OFF:
                pollenShooter.setVelocity(0);
                nectarShooter.setVelocity(0);
                break;
            case SHOOT:
                pollenShooter.setVelocity(pollenSpeed);
                nectarShooter.setVelocity(nectarSpeed);
                intake.setState(Intake.CaseModes.HALF_SPEED);
//                double errorDeg = (angleError) * (180 / Math.PI);
//                double new_joystick_rx = errorDeg * kP - velocityDeg * kD;
                //drivebase.updateAutoAim(new_joystick_rx);
                //if (Math.abs((angleError) * ((180/Math.PI))) < 1 && getPollenShooterVelocity() >= speed) {
                // Once both flywheels have reached speed, stay in the firing state until SHOOT is exited,
                // even if the speed dips as balls pass through.
                if (!shootReady && getPollenShooterVelocity() >= pollenSpeed && getNectarShooterVelocity() >= nectarSpeed) {
                    shootReady = true;
                }
                if (shootReady) {
                    gateOpen = true;
                    intake.setState(Intake.CaseModes.ON);
                    gamepad1.rumble(1000);
                }
                break;
            case SHOOT_GATE_CLOSED:
                pollenShooter.setVelocity(pollenSpeed);
                nectarShooter.setVelocity(nectarSpeed);
                // doesn't touch the intake, so the driver keeps manual intake control
                break;
            case REVERSE:
                gateOpen = true;
                intake.setState(Intake.CaseModes.REVERSE);
                pollenShooter.setVelocity(-900);
                nectarShooter.setVelocity(-900);
                break;
        }
        // The gates are written once per loop: open if the mode wants them open OR the driver is holding the manual override.
        if (gateOpen || manualGateOpen) {
            openGate();
        } else {
            closeGate();
        }
    }

    /** Driver override: while true, the gates are held open regardless of the shooter mode or flywheel speed. */
    public void setManualGateOpen(boolean open) {
        manualGateOpen = open;
    }

    public void setState(CaseModes s) {
        if (s != CaseModes.SHOOT) shootReady = false;
        currentMode = s; //currentMode = s??? what does that mean??
    }
//    public double distanceToSpeed(double distanceCm) {
//        speed = lut.getSpeed(distanceCm);
//        distance = distanceCm;
//        return speed;
//    }
    /** Stores the target speeds without spinning the motors; update() applies them in the SHOOT modes. */
    public void setTargetSpeeds(double pollenSpeed, double nectarSpeed){
        this.pollenSpeed = pollenSpeed;
        this.nectarSpeed = nectarSpeed;
    }
    public void setShooterSpeed(double pollenSpeed, double nectarSpeed){
        this.pollenSpeed = pollenSpeed;
        this.nectarSpeed = nectarSpeed;
        pollenShooter.setVelocity(pollenSpeed);
        nectarShooter.setVelocity(nectarSpeed);
    }
//    public String telemetryUpdate() {
//        return "Servo Position: " + hood.getPosition() + " \n ShooterMode: " + currentMode
//                + " \n Shooter Speed: " + getShooterVelocity() + " \n " + "Target Speed/Vel: " + distance + ":" + speed;
//    }
    public double getPollenShooterVelocity()
    {
        return pollenShooter.getVelocity();
    }
    public double getNectarShooterVelocity()
    {
        return nectarShooter.getVelocity();
    }

    double degreesToPosition(double degrees) {
        return 0.15 + (degrees - 15) / (75 - 15) * (0.70 - 0.15);
    }
    public void closeGate() {
        pollenGate.setPosition(0.175);
        nectarGate.setPosition(0.5);
    }
    public void openGate()  {
        pollenGate.setPosition(0.28);

        nectarGate.setPosition(1.0);
    }

}
