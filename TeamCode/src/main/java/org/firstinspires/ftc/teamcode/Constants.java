// DISABLED (teleop-only): uncomment when working on auto
//package org.firstinspires.ftc.teamcode;
//
//
//import com.pedropathing.control.FilteredPIDFCoefficients;
//import com.pedropathing.control.PIDFCoefficients;
//import com.pedropathing.follower.Follower;
//import com.pedropathing.follower.FollowerConstants;
//import com.pedropathing.ftc.FollowerBuilder;
//import com.pedropathing.ftc.drivetrains.MecanumConstants;
//import com.pedropathing.ftc.localization.constants.PinpointConstants;
//import com.pedropathing.paths.PathConstraints;
//import com.qualcomm.hardware.gobilda.GoBildaPinpointDriver;
//import com.qualcomm.robotcore.hardware.DcMotorSimple;
//import com.qualcomm.robotcore.hardware.HardwareMap;
//
//import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
//import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
//import org.firstinspires.ftc.robotcore.external.navigation.Pose2D;
//
//public class Constants {
//
//
//    public static final double CLOSE_SHOOT_SPEED = 1100;
//    public static final Pose2D BLUE_LAUNCH = new Pose2D(DistanceUnit.CM, -110, 10, AngleUnit.RADIANS, 0);
//    public static final Pose2D RED_LAUNCH = new Pose2D(DistanceUnit.CM, -110, 10, AngleUnit.RADIANS, 0);
//    public static final Pose2D BLUE_CENTER_GOAL = new Pose2D(DistanceUnit.CM, -157.5, -152.4, AngleUnit.RADIANS, 0); // x, y -152.4, -152.4,
//    public static final Pose2D RED_CENTER_GOAL = new Pose2D(DistanceUnit.CM, -152.4, 152.4, AngleUnit.RADIANS, 0);
//
//
//
//    public static final class Drive {
//        public static final double trackEndpointHeadingMaxDistance = 12.0;
//        public static final double calculateTargetHeadingMinDistance = 15.0;
//        public static final double defaultFollowRadius = 15;
//    }
//
//    public static class PedroPathing {
//        public static FollowerConstants followerConstants = new FollowerConstants()
//                .mass(11.5)
//                .forwardZeroPowerAcceleration(-28)
//                .lateralZeroPowerAcceleration(-55)
//                .translationalPIDFCoefficients(new PIDFCoefficients(0.15, 0, 0.02,0.01))
//                .headingPIDFCoefficients(new PIDFCoefficients(1, 0, 0.1,0.01))
//                .drivePIDFCoefficients(new FilteredPIDFCoefficients(0.02,0.0,0.00001,0.6,0.01))
//                .centripetalScaling(0.0008);
//
//        public static MecanumConstants driveConstants = new MecanumConstants()
//                .maxPower(1)
//                .leftFrontMotorName("frontLeftMotor")
//                .rightFrontMotorName("frontRightMotor")
//                .leftRearMotorName("backLeftMotor")
//                .rightRearMotorName("backRightMotor")
//                .leftFrontMotorDirection(DcMotorSimple.Direction.REVERSE)
//                .leftRearMotorDirection(DcMotorSimple.Direction.REVERSE)
//                .rightFrontMotorDirection(DcMotorSimple.Direction.FORWARD)
//                .rightRearMotorDirection(DcMotorSimple.Direction.FORWARD)
//                .xVelocity(73)//71.33699287955217
//                .yVelocity(43);
//
//        public static PinpointConstants localizerConstants = new PinpointConstants()
//                .forwardPodY(-0.597440945)
//                .strafePodX(8.0989003937)
//                .distanceUnit(DistanceUnit.CM)
//                .hardwareMapName("odo")
//                .encoderResolution(GoBildaPinpointDriver.GoBildaOdometryPods.goBILDA_4_BAR_POD)
//                .forwardEncoderDirection(GoBildaPinpointDriver.EncoderDirection.REVERSED)
//                .strafeEncoderDirection(GoBildaPinpointDriver.EncoderDirection.REVERSED);
//
//        public static PathConstraints pathConstraints = new PathConstraints(0.99,
//                100,
//                1,
//                1);
//
//        public static Follower createFollower(HardwareMap hardwareMap) {
//            return new FollowerBuilder(followerConstants, hardwareMap)
//                    .pathConstraints(pathConstraints)
//                    .mecanumDrivetrain(driveConstants)
//                    .pinpointLocalizer(localizerConstants)
//                    .build();
//        }
//    }
//}
