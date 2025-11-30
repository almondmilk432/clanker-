package org.firstinspires.ftc.teamcode.subsystems;

import dev.nextftc.control.ControlSystem;
import dev.nextftc.core.commands.Command;
import dev.nextftc.core.subsystems.Subsystem;
import dev.nextftc.hardware.controllable.MotorGroup;
import dev.nextftc.hardware.controllable.RunToVelocity;
import dev.nextftc.hardware.impl.MotorEx;
import dev.nextftc.hardware.powerable.SetPower;

public class outtake implements Subsystem {
    public static final outtake INSTANCE = new outtake();
    public outtake(){

    }


    MotorGroup motors = new MotorGroup(
            new MotorEx("outR").reversed(),
            new MotorEx("outL")
    );

    private ControlSystem controlSystem = ControlSystem.builder()
            .basicFF(11.1, 0 ,0)
            .build();




    public Command Out (){
        return new RunToVelocity(controlSystem, 2500, 1500).requires(this);
    }

    public Command Stop (){
        return new RunToVelocity(controlSystem, 0).requires(this);
    }

    public Command setp(){
        return new SetPower(motors, 0.75).requires(this);
    }

    public Command setpO (){
        return new SetPower(motors, 0).requires(this);
    }
    @Override
    public void periodic() {
        motors.setPower(controlSystem.calculate(motors.getState()));

    }
    public static double getVelocity() {
        return INSTANCE.motors.getVelocity();
    }

}
