package org.firstinspires.ftc.teamcode.subsystems;

import dev.nextftc.core.commands.Command;
import dev.nextftc.core.subsystems.Subsystem;
import dev.nextftc.hardware.impl.ServoEx;
import dev.nextftc.hardware.positionable.SetPosition;

public class shootadj implements Subsystem {
    public static final shootadj INSTANCE = new shootadj();

    private shootadj() {
    }

    private ServoEx servoL = new ServoEx("adjL");
    private ServoEx servoR = new ServoEx("adjR");

    public Command up() {
        new SetPosition(servoR, -.15).requires(this);
        return new SetPosition(servoL, .15).requires(this);
    }
    public Command mid() {
        new SetPosition(servoR, .17).requires(this);
        return new SetPosition(servoL, -.17).requires(this);
    }
    public Command low() {
        new SetPosition(servoR, 0.4).requires(this);
        return new SetPosition(servoL, -.04).requires(this);
    }
}