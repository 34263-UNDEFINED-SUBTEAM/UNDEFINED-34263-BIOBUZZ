
package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.vision.VisionPortal;
import org.firstinspires.ftc.vision.apriltag.AprilTagDetection;
import org.firstinspires.ftc.vision.apriltag.AprilTagProcessor;
import org.firstinspires.ftc.vision.apriltag.AprilTagSingleDetection;

import java.util.List;

@TeleOp(name = "AVT", group = "Test")
public class AVT extends LinearOpMode {

    private static final String CAMERA_NAME = "Webcam 1";

    @Override
    public void runOpMode() {

        AprilTagProcessor aprilTag =
                new AprilTagProcessor.Builder().build();

        VisionPortal visionPortal =
                new VisionPortal.Builder()
                        .setCamera(
                                hardwareMap.get(
                                        WebcamName.class,
                                        CAMERA_NAME
                                )
                        )
                        .addProcessor(aprilTag)
                        .build();

        telemetry.addLine("AVT - APRILTAG VISIBILITY TEST");
        telemetry.addLine("Initializing camera...");
        telemetry.update();

        while (opModeInInit()
                && visionPortal.getCameraState()
                != VisionPortal.CameraState.STREAMING) {

            telemetry.addData(
                    "Camera",
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
        telemetry.update();

        waitForStart();

        while (opModeIsActive()) {

            List<AprilTagDetection> detections =
                    aprilTag.getDetections();

            int visibleTags = 0;

            telemetry.addLine("APRILTAG VISIBILITY");
            telemetry.addData(
                    "Total detections",
                    detections.size()
            );

            for (AprilTagDetection detection : detections) {

                if (detection instanceof AprilTagSingleDetection) {

                    AprilTagSingleDetection singleDetection =
                            (AprilTagSingleDetection) detection;

                    telemetry.addData(
                            "Visible Tag",
                            "ID %d",
                            singleDetection.id
                    );

                    visibleTags++;
                }
            }

            if (visibleTags == 0) {
                telemetry.addLine("No individual AprilTags detected.");
            }

            telemetry.update();
            sleep(50);
        }

        visionPortal.close();
    }
}
