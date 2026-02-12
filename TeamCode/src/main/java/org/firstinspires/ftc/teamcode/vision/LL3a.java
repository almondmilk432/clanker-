package org.firstinspires.ftc.teamcode.vision;

import com.qualcomm.hardware.limelightvision.LLResult;
import com.qualcomm.hardware.limelightvision.Limelight3A;
import com.qualcomm.robotcore.hardware.HardwareMap;

import dev.nextftc.core.subsystems.Subsystem;
import org.firstinspires.ftc.robotcore.external.navigation.Pose3D;

public class LL3a implements Subsystem {

    public static LL3a INSTANCE;

    private Limelight3A limelight;
    private LLResult latest;
    private Pose3D mtPose;


    private final int startupPipeline;

    private LL3a(int pipeline) {
        this.startupPipeline = pipeline;
    }

    public static void init(HardwareMap hardwareMap, int pipeline) {
        INSTANCE = new LL3a(pipeline);
        INSTANCE.limelight = hardwareMap.get(Limelight3A.class, "limelight");
    }

    public void onStart() {
        limelight.pipelineSwitch(startupPipeline);
        limelight.start();
    }

    @Override
    public void periodic() {
        latest = limelight.getLatestResult();

        mtPose = latest.getBotpose();
/*
        mtPose = latest.getBotpose_MT2();
        if (latest != null && latest.isValid()) {
            pose = latest.getBotpose();
        } else {
            pose = null;
        }
        */

    }

    public boolean hasValidTarget() {
        return mtPose != null;
    }

    public Pose3D getPose() {
        return mtPose;
    }

    public double getDistance() {
        if (mtPose == null) return 0;

        double x = mtPose.getPosition().x;
        double y = mtPose.getPosition().y;
        return Math.hypot(x, y);
    }

    public double Tx() {
        return latest != null ? latest.getTx() : 0;
    }

    public double Ty() {
        return latest != null ? latest.getTy() : 0;
    }
}
