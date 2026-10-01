// DISABLED (teleop-only): uncomment when working on auto
//package org.firstinspires.ftc.teamcode.commands;
//
//import com.qualcomm.robotcore.util.ElapsedTime;
//
//import org.firstinspires.ftc.robotcore.external.Telemetry;
//import org.firstinspires.ftc.teamcode.commandSystem.Command;
//import org.firstinspires.ftc.teamcode.subsystems.Drivebase;
//
//public class BrakeCommand extends Command {
//    Telemetry telemetry;
//    private final Drivebase drivebase;
//    private final Double durationSeconds;
//    private double startTime;
//
//    public BrakeCommand(Drivebase drivebase, double durationSeconds, Telemetry telemetry) {
//        this.drivebase = drivebase;
//        this.durationSeconds = durationSeconds;
//        this.telemetry = telemetry;
//        addRequirements(this.drivebase);
//    }
//
//    @Override
//    public void initialize() {
//        timer = new ElapsedTime();
//        timer.reset();
//        startTime = timer.milliseconds();
//    }
//
//
//    @Override
//    public void execute() {
//    drivebase.brake();
//        telemetry.addData("Braking", "Brake Command");
//    }
//    @Override
//
//    public boolean isFinished() {
//        return timer.seconds() - startTime >= durationSeconds;
//    }
//    @Override
//    public void end(boolean interrupted) {
//        drivebase.end();
//    }
//}
