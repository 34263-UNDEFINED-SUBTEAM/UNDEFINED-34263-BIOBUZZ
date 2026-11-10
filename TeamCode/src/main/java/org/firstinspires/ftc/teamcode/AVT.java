
package org.firstinspires.ftc.teamcode;

import com.qualcomm.hardware.limelightvision.LLResult;
import com.qualcomm.hardware.limelightvision.LLResultTypes;
import com.qualcomm.hardware.limelightvision.Limelight3A;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import java.util.List;

@TeleOp(name = "AVT - AprilTag Vision Test", group = "Test")
public class AVT extends LinearOpMode {

    private Limelight3A limelight;

    @Override
    public void runOpMode() {

        // Limelight Ethernet device
        // IP: 179.29.0.1
        // "limelight" must match its Robot Configuration name.
        limelight = hardwareMap.get(Limelight3A.class, "limelight");

        limelight.pipelineSwitch(0);

        telemetry.addLine("AVT - LIMELIGHT CONNECTED");
        telemetry.addLine("IP: 179.29.0.1");
        telemetry.addLine("Pipeline: 0");
        telemetry.update();

        waitForStart();

        if (isStopRequested()) return;

        limelight.start();

        while (opModeIsActive()) {

            LLResult result = limelight.getLatestResult();

            if (result != null && result.isValid()) {

                List<LLResultTypes.FiducialResult> tags =
                        result.getFiducialResults();

                if (tags != null && !tags.isEmpty()) {

                    telemetry.addData("Status", "APRILTAG DETECTED");

                    for (LLResultTypes.FiducialResult tag : tags) {
                        telemetry.addData(
                                "AprilTag ID",
                                tag.getFiducialId()
                        );
                    }

                } else {
                    telemetry.addLine("No AprilTags detected");
                }

            } else {
                telemetry.addLine("Waiting for Limelight data...");
            }

            telemetry.update();
            sleep(50);
        }

        limelight.stop();
    }
}
