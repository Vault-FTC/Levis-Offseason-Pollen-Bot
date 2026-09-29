package org.firstinspires.ftc.teamcode;

import com.qualcomm.hardware.gobilda.GoBildaPinpointDriver;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.robotcore.external.navigation.Pose2D;
import org.firstinspires.ftc.teamcode.Auto.Location;

public class Drivebase {

    private final DcMotorEx frontLeftMotor, frontRightMotor, backLeftMotor, backRightMotor;
    GoBildaPinpointDriver odo;
    double headingOffsetThingy;

    double modify_joystick_rotate = 0.0;

    public Drivebase(HardwareMap hardwareMap) {
        frontLeftMotor = hardwareMap.get(DcMotorEx.class, "frontLeftMotor");
        frontRightMotor = hardwareMap.get(DcMotorEx.class, "frontRightMotor");
        backLeftMotor = hardwareMap.get(DcMotorEx.class, "backLeftMotor");
        backRightMotor = hardwareMap.get(DcMotorEx.class, "backRightMotor");

        frontLeftMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        frontRightMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        backLeftMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        backRightMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        frontLeftMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        frontRightMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        backLeftMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        backRightMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

// Reverse one side of motors if needed (depends on robot configuration)
        frontLeftMotor.setDirection(DcMotorEx.Direction.FORWARD);
        backLeftMotor.setDirection(DcMotorEx.Direction.FORWARD);
        frontRightMotor.setDirection(DcMotorSimple.Direction.FORWARD);
        backRightMotor.setDirection(DcMotorSimple.Direction.REVERSE);
// Odometry constants and such
        odo = hardwareMap.get(GoBildaPinpointDriver.class,"odo");
        //odo.setOffsets(205.71207, -15.175, DistanceUnit.MM);
        odo.setOffsets(145.0, -4.0, DistanceUnit.MM);
        odo.setEncoderResolution(GoBildaPinpointDriver.GoBildaOdometryPods.goBILDA_4_BAR_POD);
        odo.setEncoderDirections(
                GoBildaPinpointDriver.EncoderDirection.FORWARD,
                GoBildaPinpointDriver.EncoderDirection.REVERSED);
        //  odo.resetPosAndIMU();
    }

    public void setToCoastMode() {
        frontLeftMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
        frontRightMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
        backLeftMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
        backRightMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
    }
    public void resetHeading(double headingDegrees) {
        odo.setHeading(headingDegrees, AngleUnit.DEGREES);
        //odo.resetPosAndIMU();
    }

    public Pose2D getPosition() {
        return odo.getPosition();
    }

    public GoBildaPinpointDriver getOdo()
    {
        return odo;
    }



    public String getPositionTelemetry() {
        return "X offset (forwards/backwards): " + odo.getPosX(DistanceUnit.CM) + " Y (left/right): " + odo.getPosY(DistanceUnit.CM) + " Heading: " + odo.getHeading(AngleUnit.DEGREES);
    }

    public void updateAutoAim(double joystick_rx_modifier)
    {
        modify_joystick_rotate = joystick_rx_modifier;
    }

    public void drive(double forward, double right, double rotate, double headingOffset) {
        headingOffsetThingy = headingOffset;
        drive(forward, right, rotate + modify_joystick_rotate);
    }

    public void drive(double forward, double right, double rotate) {
        double botHeading = -odo.getHeading(AngleUnit.RADIANS) + headingOffsetThingy;
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
    public void brake() {
        frontLeftMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        frontRightMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        backLeftMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        backRightMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
    }

    public void end() {
        frontLeftMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
        frontRightMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
        backLeftMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
        backRightMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
    }

    public boolean isAtPosition(Location target) {
        return isAtPosition(target, 15, 17.5);
    }

    public boolean isAtPosition(Location target, double toleranceXY, double toleranceAngle) {
        double currentX = odo.getPosX(DistanceUnit.CM);
        double currentY = odo.getPosY(DistanceUnit.CM);
        double currentHeading = odo.getHeading(AngleUnit.DEGREES);
        return Math.abs(currentY - target.Strafe) < toleranceXY &&
                Math.abs(currentX - target.Forward) < toleranceXY &&
                Math.abs(currentHeading-target.TurnDegrees) < toleranceAngle;
    }

    public void setCurrentPose(Pose2D pos)
    {
        odo.setPosition(pos);
    }

    public void setCurrentPose(double x, double y, double radians)
    {
        setCurrentPose(new Pose2D(DistanceUnit.CM, x, y, AngleUnit.RADIANS, radians));
    }

    public void setCurrentPose(double x, double y)
    {
        odo.setPosX(x, DistanceUnit.CM);
        odo.setPosY(y, DistanceUnit.CM);
    }

    public void update() {
        odo.update(); // updates the odometry internally
    }

    public static double distanceToGoal(Pose2D robot, Pose2D goal) {
        double dx = robot.getX(DistanceUnit.CM) - goal.getX(DistanceUnit.CM);
        double dy = robot.getY(DistanceUnit.CM) - goal.getY(DistanceUnit.CM);
        return Math.hypot(dx, dy);
    }
    public static double angleToGoal(Pose2D robot, Pose2D goal) {
        double dx = goal.getX(DistanceUnit.CM) - robot.getX(DistanceUnit.CM);
        double dy = goal.getY(DistanceUnit.CM) - robot.getY(DistanceUnit.CM);
        double goalHeading = Math.atan2(dy, dx); // radians
        double robotHeading = robot.getHeading(AngleUnit.RADIANS);
        double error = goalHeading - robotHeading;

        // Normalize to [-π, π]
        while (error > Math.PI) error -= 2 * Math.PI;
        while (error < -Math.PI) error += 2 * Math.PI;

        return error;
    }

}