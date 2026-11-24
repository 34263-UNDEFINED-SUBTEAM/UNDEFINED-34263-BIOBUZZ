
package org.firstinspires.ftc.teamcode;

import com.qualcomm.hardware.gobilda.GoBildaPinpointDriver;
import com.qualcomm.hardware.limelightvision.LLResult;
import com.qualcomm.hardware.limelightvision.Limelight3A;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;

@Autonomous(name = "AMLT - Hardware Diagnostics", group = "AMLT")
public class AMLT extends LinearOpMode {

    private DcMotor frontLeft;
    private DcMotor frontRight;
    private DcMotor backLeft;
    private DcMotor backRight;

    private GoBildaPinpointDriver pinpoint;
    private Limelight3A limelight;

    private boolean pinpointReady = false;
    private boolean limelightReady = false;

    private String driveStatus = "NOT CHECKED";
    private String pinpointStatus = "NOT CHECKED";
    private String limelightStatus = "NOT CHECKED";

    @Override
    public void runOpMode() {

        telemetry.addLine("AMLT - AUTO MACHINE LEARNING TEST");
        telemetry.addLine("Team 34263 - Undefined");
        telemetry.addLine("Initializing hardware...");
        telemetry.update();

        initializeDrive();
        initializePinpoint();
        initializeLimelight();

        telemetry.clearAll();
        telemetry.addLine("========== AMLT PREFLIGHT ==========");
        telemetry.addData("Drive", driveStatus);
        telemetry.addData("Pinpoint", pinpointStatus);
        telemetry.addData("Limelight", limelightStatus);
        telemetry.addLine("------------------------------------");
        telemetry.addLine("Robot movement is DISABLED.");
        telemetry.update();

        waitForStart();

        if (isStopRequested()) {
            stopAll();
            return;
        }

        while (opModeIsActive()) {

            telemetry.clearAll();

            telemetry.addLine("========== AMLT LIVE DIAGNOSTICS ==========");

            telemetry.addData("Drive", driveStatus);
            telemetry.addData("Pinpoint", pinpointStatus);
            telemetry.addData("Limelight", limelightStatus);

            if (pinpointReady) {
                try {
                    pinpoint.update();

                    double x = pinpoint.getPosX(DistanceUnit.INCH);
                    double y = pinpoint.getPosY(DistanceUnit.INCH);

                    double heading =
                            pinpoint.getHeading(AngleUnit.DEGREES);

                    telemetry.addLine("--- PINPOINT DATA ---");
                    telemetry.addData("X", "%.2f in", x);
                    telemetry.addData("Y", "%.2f in", y);
                    telemetry.addData("Heading", "%.2f deg", heading);

                } catch (Exception e) {
                    pinpointStatus = "READ ERROR: " + e.getMessage();
                    pinpointReady = false;
                }
            }

            if (limelightReady) {
                try {
                    LLResult result = limelight.getLatestResult();

                    telemetry.addLine("--- LIMELIGHT DATA ---");

                    if (result != null && result.isValid()) {
                        telemetry.addData("Target", "VALID");
                        telemetry.addData("TX", "%.2f", result.getTx());
                        telemetry.addData("TY", "%.2f", result.getTy());
                    } else {
                        telemetry.addData("Target", "No valid target");
                    }

                } catch (Exception e) {
                    limelightStatus = "READ ERROR: " + e.getMessage();
                    limelightReady = false;
                }
            }

            telemetry.addLine("------------------------------------");
            telemetry.addLine("DIAGNOSTICS ONLY - NO MOVEMENT");
            telemetry.update();

            sleep(50);
        }

        stopAll();
    }

    private void initializeDrive() {
        try {
            frontLeft = hardwareMap.get(
                    DcMotor.class, "front_left_drive");

            frontRight = hardwareMap.get(
                    DcMotor.class, "front_right_drive");

            backLeft = hardwareMap.get(
                    DcMotor.class, "back_left_drive");

            backRight = hardwareMap.get(
                    DcMotor.class, "back_right_drive");

            stopDrive();

            driveStatus = "ALL FOUR MOTORS FOUND";

        } catch (Exception e) {
            driveStatus = "FAILED: " + e.getMessage();
        }
    }

    private void initializePinpoint() {
        try {
            pinpoint = hardwareMap.get(
                    GoBildaPinpointDriver.class, "pinpoint");

            pinpointReady = true;
            pinpointStatus = "DEVICE FOUND";

        } catch (Exception e) {
            pinpointStatus = "NOT FOUND: " + e.getMessage();
        }
    }

    private void initializeLimelight() {
        try {
            limelight = hardwareMap.get(
                    Limelight3A.class, "limelight");

            limelight.start();

            limelightReady = true;
            limelightStatus = "DEVICE FOUND - STARTED";

        } catch (Exception e) {
            limelightStatus = "NOT FOUND: " + e.getMessage();
        }
    }

    private void stopDrive() {
        if (frontLeft != null) frontLeft.setPower(0);
        if (frontRight != null) frontRight.setPower(0);
        if (backLeft != null) backLeft.setPower(0);
        if (backRight != null) backRight.setPower(0);
    }

    private void stopAll() {
        stopDrive();

        if (limelight != null) {
            limelight.stop();
        }
    }
}
