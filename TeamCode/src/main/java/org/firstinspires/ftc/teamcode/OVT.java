
package org.firstinspires.ftc.teamcode;

import com.qualcomm.hardware.limelightvision.LLResult;
import com.qualcomm.hardware.limelightvision.LLResultTypes;
import com.qualcomm.hardware.limelightvision.Limelight3A;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.util.ElapsedTime;

import java.util.List;
import java.util.Locale;

@Autonomous(name = "OVT - Object Vision Test", group = "Test")
public class OVT extends LinearOpMode {

    private Limelight3A limelight;

    private DcMotor leftDrive;
    private DcMotor rightDrive;

    // Limelight pipeline containing the trained object detector.
    private static final int OBJECT_PIPELINE = 1;

    // Minimum detector confidence.
    private static final double MIN_CONFIDENCE = 60.0;

    // Conservative test movement.
    private static final double MOVEMENT_POWER = 0.20;
    private static final double MOVEMENT_SECONDS = 0.20;

    // Time an object must disappear before another action.
    private static final double CLEAR_SECONDS = 1.0;

    private final ElapsedTime movementTimer = new ElapsedTime();
    private final ElapsedTime clearTimer = new ElapsedTime();

    private boolean actionLocked = false;

    @Override
    public void runOpMode() {

        // Initialize drive motors.
        leftDrive = hardwareMap.get(
                DcMotor.class, "left_drive");

        rightDrive = hardwareMap.get(
                DcMotor.class, "right_drive");

        leftDrive.setDirection(DcMotor.Direction.REVERSE);
        rightDrive.setDirection(DcMotor.Direction.FORWARD);

        leftDrive.setZeroPowerBehavior(
                DcMotor.ZeroPowerBehavior.BRAKE);

        rightDrive.setZeroPowerBehavior(
                DcMotor.ZeroPowerBehavior.BRAKE);

        stopDrive();

        // Initialize Limelight.
        limelight = hardwareMap.get(
                Limelight3A.class, "limelight");

        limelight.pipelineSwitch(OBJECT_PIPELINE);
        limelight.start();

        telemetry.addLine("OVT - Object Vision Test");
        telemetry.addData("Pipeline", OBJECT_PIPELINE);
        telemetry.addData("Limelight Connected",
                limelight.isConnected());
        telemetry.addLine("Waiting for start...");
        telemetry.update();

        waitForStart();

        clearTimer.reset();

        try {

            while (opModeIsActive()) {

                LLResult result = limelight.getLatestResult();

                boolean recognizedObject = false;

                if (result != null && result.isValid()) {

                    List<LLResultTypes.DetectorResult> detections =
                            result.getDetectorResults();

                    for (LLResultTypes.DetectorResult detection
                            : detections) {

                        String className =
                                detection.getClassName();

                        double confidence =
                                detection.getConfidence();

                        telemetry.addData(
                                "Raw Detection",
                                "%s (%.1f%%)",
                                className,
                                confidence
                        );

                        if (confidence < MIN_CONFIDENCE) {
                            continue;
                        }

                        String objectType =
                                identifyObject(className);

                        if (!objectType.equals("UNKNOWN")) {

                            recognizedObject = true;

                            telemetry.addData(
                                    "Recognized Object",
                                    objectType
                            );

                            if (!actionLocked) {

                                actionLocked = true;

                                executeObjectAction(objectType);
                            }

                            break;
                        }
                    }

                } else {

                    telemetry.addLine(
                            "Waiting for Limelight detection...");
                }

                // Re-arm after the object disappears.
                if (!recognizedObject) {

                    if (clearTimer.seconds() >= CLEAR_SECONDS) {
                        actionLocked = false;
                    }

                } else {

                    clearTimer.reset();
                }

                telemetry.addData(
                        "Action Locked",
                        actionLocked
                );

                telemetry.update();

                idle();
            }

        } finally {

            stopDrive();

            if (limelight != null) {
                limelight.stop();
            }
        }
    }

    private String identifyObject(String name) {

        if (name == null) {
            return "UNKNOWN";
        }

        String normalized = name
                .toLowerCase(Locale.US)
                .replace("_", "")
                .replace("-", "")
                .replace(" ", "");

        if (normalized.contains("pollen")) {
            return "POLLEN";
        }

        if (normalized.contains("red")
                && normalized.contains("nectar")) {
            return "RED_NECTAR";
        }

        if (normalized.contains("blue")
                && normalized.contains("nectar")) {
            return "BLUE_NECTAR";
        }

        return "UNKNOWN";
    }

    private void executeObjectAction(String objectType) {

        stopDrive();

        telemetry.addData("Executing", objectType);
        telemetry.update();

        // Speak through the REV Driver Hub.
        telemetry.speak("Siddarth is the Best!");

        switch (objectType) {

            case "POLLEN":

                // Short forward movement.
                moveRobot(
                        MOVEMENT_POWER,
                        MOVEMENT_POWER
                );

                break;

            case "RED_NECTAR":

                // Short left turn.
                moveRobot(
                        -MOVEMENT_POWER,
                        MOVEMENT_POWER
                );

                break;

            case "BLUE_NECTAR":

                // Short right turn.
                moveRobot(
                        MOVEMENT_POWER,
                        -MOVEMENT_POWER
                );

                break;

            default:

                stopDrive();
                break;
        }

        stopDrive();
    }

    private void moveRobot(
            double leftPower,
            double rightPower) {

        movementTimer.reset();

        leftDrive.setPower(leftPower);
        rightDrive.setPower(rightPower);

        while (opModeIsActive()
                && movementTimer.seconds() < MOVEMENT_SECONDS) {

            idle();
        }

        stopDrive();
    }

    private void stopDrive() {

        if (leftDrive != null) {
            leftDrive.setPower(0);
        }

        if (rightDrive != null) {
            rightDrive.setPower(0);
        }
    }
}
