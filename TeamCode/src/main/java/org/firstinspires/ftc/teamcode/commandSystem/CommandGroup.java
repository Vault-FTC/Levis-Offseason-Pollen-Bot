package org.firstinspires.ftc.teamcode.commandSystem;

public abstract class CommandGroup extends Command {
    final Command[] commands;

    public CommandGroup(Command... commands) {
        this.commands = commands;
    }
}