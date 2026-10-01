// DISABLED (teleop-only): uncomment when working on auto
//package org.firstinspires.ftc.teamcode.commands;
//
//import com.qualcomm.robotcore.util.ElapsedTime;
//
//import org.firstinspires.ftc.robotcore.external.Telemetry;
//import org.firstinspires.ftc.teamcode.commandSystem.Command;
//import org.firstinspires.ftc.teamcode.subsystems.Intake;
//import org.firstinspires.ftc.teamcode.subsystems.Shooter;
//
//public class IntakeCommand extends Command {
//    Telemetry telemetry;
//    private final Intake intake;
//    private final Shooter shooter;
//    private final double durationMs;
//    private double startTime;
//
//    public IntakeCommand(Intake intake, double durationSeconds, Telemetry telemetry, Shooter shooter) {
//        this.intake = intake;
//        this.durationMs = durationSeconds * 1000;
//        this.telemetry = telemetry;
//        this.shooter = shooter;
//        addRequirements(this.intake, this.shooter);
//    }
//
//    @Override
//    public void initialize() {
//        timer = new ElapsedTime();
//        timer.reset();
//        startTime = timer.milliseconds();
//    }
//
//    @Override
//    public void execute() {
//        intake.setState(Intake.CaseModes.ON);
//        shooter.closeGate();
//        telemetry.addData("Running", "Intake Command");
//    }
//
//    @Override
//    public boolean isFinished() {
//        return timer.milliseconds() - startTime >= durationMs;
//    }
//
//    @Override
//    public void end(boolean interrupted) {
//        intake.setState(Intake.CaseModes.OFF);
//    }
//}
