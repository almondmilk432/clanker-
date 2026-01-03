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
    private Pose3D pose;

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

        if (latest != null && latest.isValid()) {
            pose = latest.getBotpose();   // MT1
        } else {
            pose = null;
        }
    }

    public boolean hasValidTarget() {
        return pose != null;
    }

    public Pose3D getPose() {
        return pose;
    }

    public double getDistance() {
        if (pose == null) return 0;
        return Math.hypot(
                pose.getPosition().x,
                pose.getPosition().y
        );
    }

    public double getDistanceToTag() {
        return latest != null ? latest.getTx() : 0;
    }

    public double getTy() {
        return latest != null ? latest.getTy() : 0;
    }
}
