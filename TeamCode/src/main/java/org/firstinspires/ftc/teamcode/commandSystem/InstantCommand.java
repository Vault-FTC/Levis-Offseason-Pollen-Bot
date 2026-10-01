package org.firstinspires.ftc.teamcode.commandSystem;

public class InstantCommand extends Command {

    Runnable toRun;

    public InstantCommand(Subsystem subsystem, Runnable toRun) {
        addRequirements(subsystem);
        this.toRun = toRun;
    }
    public InstantCommand(Runnable toRun) {
        this.toRun = toRun;
    }

    public InstantCommand() {
        this(() -> {

        });
    }

    @Override
    public void execute() {
        toRun.run();
    }

}