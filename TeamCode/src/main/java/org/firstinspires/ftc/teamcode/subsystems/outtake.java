package org.firstinspires.ftc.teamcode.subsystems;

import dev.nextftc.control.ControlSystem;
import dev.nextftc.core.commands.Command;
import dev.nextftc.core.subsystems.Subsystem;
import dev.nextftc.hardware.controllable.MotorGroup;
import dev.nextftc.hardware.controllable.RunToVelocity;
import dev.nextftc.hardware.impl.MotorEx;

public class outtake implements Subsystem {
    public static final outtake INSTANCE = new outtake();
    public outtake(){

    }



    MotorGroup motors = new MotorGroup(
            new MotorEx("outL"),
            new MotorEx("outR").reversed()
    );



    private ControlSystem controlSystem = ControlSystem.builder()
            .velPid(0.005, 0, 0)
            .basicFF(0.001, 0.02, 0.03)
            .build();



    public Command targetVel(double rpm) {
        return new RunToVelocity(controlSystem, rpm).requires(this);
    }



    public Command Outf(){
        return new RunToVelocity(controlSystem, 1750).requires(this);
    }

    public Command Outc(){
        return new RunToVelocity(controlSystem, 1300).requires(this);
    }

    public Command Outs(){
        return new RunToVelocity(controlSystem, 1000).requires (this);
    }
    public Command Stop (){
        return new RunToVelocity(controlSystem, 0).requires(this);
    }

    @Override
    public void periodic() {
        motors.setPower(controlSystem.calculate(motors.getState()));

    }
    public static double getVelocity() {
        return INSTANCE.motors.getVelocity();
    }

}
