
package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.vision.VisionPortal;
import org.firstinspires.ftc.vision.apriltag.AprilTagDetection;
import org.firstinspires.ftc.vision.apriltag.AprilTagProcessor;

import java.util.List;

@TeleOp(name = "AVT", group = "Test")
public class AVT extends LinearOpMode {

    private AprilTagProcessor aprilTag;
    private VisionPortal visionPortal;

    // Must exactly match the camera name in Robot Configuration.
    private static final String CAMERA_NAME = "Webcam 1";

    @Override
    public void runOpMode() {

        // Create AprilTag processor.
        aprilTag = new AprilTagProcessor.Builder()
                .build();

        // Connect camera to processor.
        visionPortal = new VisionPortal.Builder()
                .setCamera(
                        hardwareMap.get(
                                WebcamName.class,
                                CAMERA_NAME
                        )
                )
                .addProcessor(aprilTag)
                .build();

        telemetry.addLine("AVT - APRILTAG VISIBILITY TEST");
        telemetry.addLine("Camera initializing...");
        telemetry.update();

        // Wait for camera streaming.
        while (opModeInInit()
                && visionPortal.getCameraState()
                != VisionPortal.CameraState.STREAMING) {

            telemetry.addData(
                    "Camera State",
                    visionPortal.getCameraState()
            );

            telemetry.update();
            sleep(50);
        }

        if (isStopRequested()) {
            visionPortal.close();
            return;
        }

        telemetry.addLine("Camera ready!");
        telemetry.addLine("Press START to detect AprilTags.");
        telemetry.update();

        waitForStart();

        // Continuously report visible AprilTags.
        while (opModeIsActive()) {

            List<AprilTagDetection> detections =
                    aprilTag.getDetections();

            telemetry.addData(
                    "Visible AprilTags",
                    detections.size()
            );

            if (detections.isEmpty()) {

                telemetry.addLine("No AprilTags detected.");

            } else {

                telemetry.addLine("Detected Tags:");

                for (AprilTagDetection detection : detections) {

                    if (detection.metadata != null) {

                        telemetry.addData(
                                "Tag ID " + detection.id,
                                detection.metadata.name
                        );

                    } else {

                        telemetry.addData(
                                "Tag ID " + detection.id,
                                "Visible (unknown metadata)"
                        );
                    }
                }
            }

            telemetry.update();
            sleep(50);
        }

        // Shut down camera.
        visionPortal.close();
    }
}
