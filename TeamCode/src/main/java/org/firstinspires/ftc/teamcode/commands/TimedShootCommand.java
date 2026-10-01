// DISABLED (teleop-only): uncomment when working on auto
//package org.firstinspires.ftc.teamcode.commands;
//
//import org.firstinspires.ftc.robotcore.external.Telemetry;
//import org.firstinspires.ftc.teamcode.commandSystem.Command;
//import org.firstinspires.ftc.teamcode.subsystems.Intake;
//import org.firstinspires.ftc.teamcode.subsystems.Shooter;
//
//public class TimedShootCommand extends Command {
//    Telemetry telemetry;
//    Shooter shooter;
//    Intake intake;
//    private final double durationMs;
//    private double startTime;
//    Intake.CaseModes intakeSetting;
//    double currentTime = 0;
//    double pollenSpeed;
//    double nectarSpeed;
//
//    public TimedShootCommand(Shooter shooter, Intake intake, double durationSeconds, Telemetry telemetry, double pollenSpeed, double nectarSpeed, double intakeSpeed) {
//        this.shooter = shooter;
//        this.intake = intake;
//        this.telemetry = telemetry;
//        this.pollenSpeed = pollenSpeed;
//        this.nectarSpeed = nectarSpeed;
//        this.durationMs = durationSeconds * 1000;
//        if(intakeSpeed > 0.8)
//        {
//            intakeSetting = Intake.CaseModes.ON;
//        }
//        else
//        {
//            intakeSetting = Intake.CaseModes.SIXTY_PERCENT_SPEED;
//        }
//        addRequirements(this.shooter, this.intake);
//    }
//
//    @Override
//    public void initialize() {
//        shooter.setShooterSpeed(pollenSpeed, nectarSpeed);
//        timer.reset();
//        startTime = timer.milliseconds();
//    }
//    @Override
//    public void execute() {
//        currentTime = timer.milliseconds();
//        double elapsed = currentTime - startTime;
//
//        shooter.openGate();
//        if (shooter.getPollenShooterVelocity() >= pollenSpeed && shooter.getNectarShooterVelocity() >= nectarSpeed) {
//            intake.setState(intakeSetting);
//        } else {
//            shooter.setShooterSpeed(pollenSpeed, nectarSpeed);
//        }
//        telemetry.addData("Running", "Shoot Command");
//    }
//
//    public boolean isFinished() {
//        return timer.milliseconds() - startTime >= durationMs;
//    }
//
//    @Override
//    public void end(boolean interrupted) {
//        intake.setState(Intake.CaseModes.OFF);
//    //    intake.spinKicker(0);
//    }
//
//}
