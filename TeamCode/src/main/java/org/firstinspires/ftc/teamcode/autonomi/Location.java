package org.firstinspires.ftc.teamcode.autonomi;

public class Location {
    public double Forward;
    public double Strafe;
    public double TurnDegrees;

    public Location(double x, double y, double turnDegrees)
    {
        Forward = x;
        Strafe = y;
        TurnDegrees = turnDegrees;
    }
    public Location(int forward, int strafe) {
        this(forward, strafe, 0);
    }
}
