package org.firstinspires.ftc.teamcode.commandSystem;

import java.util.function.BooleanSupplier;

public class DelayCommand extends Command {

    BooleanSupplier condition;

    public DelayCommand(BooleanSupplier condition) {
        this.condition = condition;
    }

    @Override
    public void execute() {

    }

    @Override
    public boolean isFinished() {
        return condition.getAsBoolean();
    }
}