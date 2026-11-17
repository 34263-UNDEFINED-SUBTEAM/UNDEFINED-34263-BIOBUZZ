
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

        // Connect to the configured Limelight Ethernet Device.
        // This name MUST match Robot Configuration.
        limelight = hardwareMap.get(Limelight3A.class, "limelight");

        telemetry.setMsTransmissionInterval(50);

        // Select Pipeline 0.
        limelight.pipelineSwitch(0);

        // Start receiving vision results.
        limelight.start();

        telemetry.addLine("================================");
        telemetry.addLine("       AVT - VISION TEST");
        telemetry.addLine("================================");
        telemetry.addData("Limelight", "Initialized");
        telemetry.addData("Pipeline", 0);
        telemetry.addLine("Press PLAY to begin.");
        telemetry.update();

        waitForStart();

        try {

            while (opModeIsActive()) {

                LLResult result = limelight.getLatestResult();

                telemetry.addLine("====== AVT STATUS ======");

                telemetry.addData(
                        "Connected",
                        limelight.isConnected()
                );

                if (result != null && result.isValid()) {

                    List<LLResultTypes.FiducialResult> tags =
                            result.getFiducialResults();

                    telemetry.addData(
                            "Pipeline",
                            result.getPipelineIndex()
                    );

                    telemetry.addData(
                            "Pipeline Type",
                            result.getPipelineType()
                    );

                    if (tags != null && !tags.isEmpty()) {

                        telemetry.addData(
                                "Detection",
                                "APRILTAG FOUND"
                        );

                        telemetry.addData(
                                "Total Tags",
                                tags.size()
                        );

                        for (int i = 0; i < tags.size(); i++) {

                            LLResultTypes.FiducialResult tag =
                                    tags.get(i);

                            telemetry.addData(
                                    "Tag " + (i + 1),
                                    "ID: %d | Family: %s",
                                    tag.getFiducialId(),
                                    tag.getFamily()
                            );

                            telemetry.addData(
                                    "Tag " + (i + 1) + " X",
                                    "%.2f degrees",
                                    tag.getTargetXDegrees()
                            );

                            telemetry.addData(
                                    "Tag " + (i + 1) + " Y",
                                    "%.2f degrees",
                                    tag.getTargetYDegrees()
                            );
                        }

                    } else {

                        telemetry.addData(
                                "Detection",
                                "NO APRILTAGS FOUND"
                        );
                    }

                } else {

                    telemetry.addData(
                            "Detection",
                            "WAITING FOR VALID RESULTS"
                    );
                }

                telemetry.update();

                sleep(50);
            }

        } finally {

            // Stop Limelight polling when OpMode ends.
            limelight.stop();
        }
    }
}
