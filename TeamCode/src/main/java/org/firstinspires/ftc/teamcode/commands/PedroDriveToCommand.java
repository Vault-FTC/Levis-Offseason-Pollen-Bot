// DISABLED (teleop-only): uncomment when working on auto
//package org.firstinspires.ftc.teamcode.commands;
//
//import com.pedropathing.follower.Follower;
//import com.pedropathing.paths.Path;
//import com.qualcomm.robotcore.util.ElapsedTime;
//
//import org.firstinspires.ftc.robotcore.external.Telemetry;
//import org.firstinspires.ftc.teamcode.commandSystem.Command;
//
//public class PedroDriveToCommand extends Command {
//
//    Telemetry telemetry;
//    private final Follower follower;
//    private final Path path;
//    double timeout;
//    ElapsedTime elapsedTime = new ElapsedTime();
//
//    public PedroDriveToCommand(Follower follower, Path path, double timeout, Telemetry telemetry) {
//        this.follower = follower;
//        this.path = path;
//        this.timeout = timeout;
//        this.telemetry = telemetry;
//    }
//
//    @Override
//    public void initialize() {
//        follower.follow(path);
//        elapsedTime.reset();
//    }
//
//    @Override
//    public void execute() {
//        follower.update();
//        telemetry.addData("Running", "Pedro DriveTo Command");
//    }
//
//    @Override
//    public boolean isFinished() {
//        return !follower.isBusy() || elapsedTime.seconds() >= timeout;
//    }
//
//    @Override
//    public void end(boolean interrupted) {
//        follower.stop();
//    }
//}
