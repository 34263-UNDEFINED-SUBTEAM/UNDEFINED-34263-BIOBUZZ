
package org.firstinspires.ftc.teamcode;

import com.qualcomm.hardware.limelightvision.LLResult;
import com.qualcomm.hardware.limelightvision.LLResultTypes;
import com.qualcomm.hardware.limelightvision.Limelight3A;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import java.util.List;

@TeleOp(name = "OVT - Object Vision Test", group = "Vision")
public class OVT extends LinearOpMode {

    private static final int PIPELINE = 1;

    private Limelight3A limelight;

    @Override
    public void runOpMode() {

        // Initialize Limelight
        limelight = hardwareMap.get(Limelight3A.class, "limelight");

        limelight.setPollRateHz(100);
        limelight.pipelineSwitch(PIPELINE);
        limelight.start();

        telemetry.addLine("================================");
        telemetry.addLine("       OBJECT VISION TEST");
        telemetry.addLine("================================");
        telemetry.addData("Pipeline", PIPELINE);
        telemetry.addLine("Limelight initialized.");
        telemetry.addLine("Waiting for START...");
        telemetry.update();

        waitForStart();

        boolean previousBallDetected = false;

        while (opModeIsActive()) {

            LLResult result = limelight.getLatestResult();

            boolean ballDetected = false;
            String detectedClass = "None";
            double confidence = 0.0;

            if (result != null && result.isValid()) {

                List<LLResultTypes.DetectorResult> detections =
                        result.getDetectorResults();

                if (detections != null && !detections.isEmpty()) {

                    ballDetected = true;

                    // Display the first detected object
                    LLResultTypes.DetectorResult detection = detections.get(0);

                    detectedClass = detection.getClassName();
                    confidence = detection.getConfidence();
                }
            }

            telemetry.clearAll();

            if (ballDetected) {

                // BIG DRIVER STATION ALERT
                telemetry.addLine("################################");
                telemetry.addLine("#                              #");
                telemetry.addLine("#    SIDDARTH IS THE BEST!     #");
                telemetry.addLine("#                              #");
                telemetry.addLine("################################");

                telemetry.addLine("");
                telemetry.addLine("          BALL FOUND!");
                telemetry.addLine("");

                telemetry.addData("Object", detectedClass);
                telemetry.addData("Confidence", "%.1f%%", confidence);

                // Speak only once when a new detection appears
                if (!previousBallDetected) {
                    telemetry.speak("Siddarth is the best!");
                }

            } else {

                telemetry.addLine("================================");
                telemetry.addLine("       OBJECT VISION TEST");
                telemetry.addLine("================================");
                telemetry.addLine("");
                telemetry.addLine("SEARCHING FOR BALL...");
                telemetry.addLine("");
                telemetry.addLine("No object detected.");
            }

            telemetry.update();

            previousBallDetected = ballDetected;

            sleep(30);
        }

        // Shut down Limelight cleanly
        limelight.stop();
    }
}
