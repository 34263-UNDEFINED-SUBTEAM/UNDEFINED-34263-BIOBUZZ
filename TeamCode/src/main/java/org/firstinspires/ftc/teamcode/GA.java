
package org.firstinspires.ftc.teamcode;

import com.qualcomm.hardware.limelightvision.LLResult;
import com.qualcomm.hardware.limelightvision.LLResultTypes;
import com.qualcomm.hardware.limelightvision.Limelight3A;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.util.ElapsedTime;

import java.util.List;

@Autonomous(name = "Garden Auto", group = "Competition")
public class GA extends LinearOpMode {

    private DcMotor frontLeft;
    private DcMotor frontRight;
    private DcMotor backLeft;
    private DcMotor backRight;
    private DcMotor intake;

    private Limelight3A limelight;

    private static final double TICKS_PER_INCH = 41.81;
    private static final double TURN_DIAMETER_INCHES = 28.0;

    private static final double DRIVE_POWER = 0.75;
    private static final double TURN_POWER = 0.35;
    private static final double INTAKE_POWER = 1.0;

    private static final double TAG_LOSS_GRACE_SECONDS = 0.4;

    private static final int[] TARGET_TAGS = {
            38, 40, 30, 32
    };

    private String stage = "Starting";

    @Override
    public void runOpMode() throws InterruptedException {

        try {
            stage = "Initializing drive motors";

            frontLeft = hardwareMap.get(
                    DcMotor.class, "front_left_drive");
            frontRight = hardwareMap.get(
                    DcMotor.class, "front_right_drive");
            backLeft = hardwareMap.get(
                    DcMotor.class, "back_left_drive");
            backRight = hardwareMap.get(
                    DcMotor.class, "back_right_drive");

            stage = "Initializing intake";

            intake = hardwareMap.get(
                    DcMotor.class, "intake");

            stage = "Initializing Limelight";

            limelight = hardwareMap.get(
                    Limelight3A.class, "limelight");

            configureDrive();

            intake.setPower(0);

            // Select the correct AprilTag pipeline on the Limelight.
            // Pipeline index is intentionally not guessed here.
            limelight.start();

            telemetry.addLine("GARDEN AUTO READY");
            telemetry.addLine("Forward 4.6 feet");
            telemetry.addLine("Right 90 degrees");
            telemetry.addLine("Backward 2 feet");
            telemetry.addLine("Watch tags 38, 40, 30, 32");
            telemetry.update();

            waitForStart();

            if (isStopRequested()) {
                return;
            }

            stage = "Forward 4.6 feet";

            if (!driveInches(55.2, DRIVE_POWER, 14.0)) {
                return;
            }

            stage = "Right turn 90 degrees";

            if (!turnDegrees(90.0, TURN_POWER, 7.0)) {
                return;
            }

            sleep(250);

            stage = "Backward 2 feet";

            if (!driveInches(-24.0, DRIVE_POWER, 9.0)) {
                return;
            }

            stage = "Waiting for target AprilTags";

            waitForTargetTags();

            stage = "Garden Auto complete";

            telemetry.addLine("GARDEN AUTO COMPLETE");
            telemetry.update();

        } catch (Exception e) {

            stopAll();

            telemetry.addLine("GARDEN AUTO ERROR");
            telemetry.addData("Stage", stage);
            telemetry.addData("Error", e.toString());
            telemetry.update();

            while (opModeIsActive()) {
                idle();
            }

        } finally {

            stopAll();

            if (limelight != null) {
                limelight.stop();
            }
        }
    }

    private void configureDrive() {

        frontLeft.setDirection(DcMotor.Direction.REVERSE);
        backLeft.setDirection(DcMotor.Direction.REVERSE);

        frontRight.setDirection(DcMotor.Direction.FORWARD);
        backRight.setDirection(DcMotor.Direction.FORWARD);

        resetEncoder(frontLeft);
        resetEncoder(frontRight);
        resetEncoder(backLeft);
        resetEncoder(backRight);
    }

    private void resetEncoder(DcMotor motor) {

        motor.setPower(0);

        motor.setMode(
                DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        motor.setMode(
                DcMotor.RunMode.RUN_USING_ENCODER);
    }

    private boolean driveInches(
            double inches,
            double power,
            double timeoutSeconds) {

        int ticks = (int) Math.round(
                inches * TICKS_PER_INCH);

        return moveRelative(
                ticks, ticks, ticks, ticks,
                power, timeoutSeconds);
    }

    // Positive degrees = clockwise/right.
    // Negative degrees = counterclockwise/left.
    private boolean turnDegrees(
            double degrees,
            double power,
            double timeoutSeconds) {

        int ticks = (int) Math.round(
                Math.PI
                        * TURN_DIAMETER_INCHES
                        * (degrees / 360.0)
                        * TICKS_PER_INCH);

        return moveRelative(
                ticks, -ticks, ticks, -ticks,
                power, timeoutSeconds);
    }

    private boolean moveRelative(
            int flTicks,
            int frTicks,
            int blTicks,
            int brTicks,
            double power,
            double timeoutSeconds) {

        if (!opModeIsActive()) {
            return false;
        }

        // Set target positions before RUN_TO_POSITION.
        frontLeft.setTargetPosition(
                frontLeft.getCurrentPosition() + flTicks);

        frontRight.setTargetPosition(
                frontRight.getCurrentPosition() + frTicks);

        backLeft.setTargetPosition(
                backLeft.getCurrentPosition() + blTicks);

        backRight.setTargetPosition(
                backRight.getCurrentPosition() + brTicks);

        frontLeft.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        frontRight.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        backLeft.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        backRight.setMode(DcMotor.RunMode.RUN_TO_POSITION);

        double safePower = Math.abs(power);

        frontLeft.setPower(safePower);
        frontRight.setPower(safePower);
        backLeft.setPower(safePower);
        backRight.setPower(safePower);

        ElapsedTime timer = new ElapsedTime();

        while (opModeIsActive()
                && timer.seconds() < timeoutSeconds
                && anyDriveMotorBusy()) {

            telemetry.addData("Stage", stage);

            telemetry.addData("FL", "%d / %d",
                    frontLeft.getCurrentPosition(),
                    frontLeft.getTargetPosition());

            telemetry.addData("FR", "%d / %d",
                    frontRight.getCurrentPosition(),
                    frontRight.getTargetPosition());

            telemetry.addData("BL", "%d / %d",
                    backLeft.getCurrentPosition(),
                    backLeft.getTargetPosition());

            telemetry.addData("BR", "%d / %d",
                    backRight.getCurrentPosition(),
                    backRight.getTargetPosition());

            telemetry.update();

            idle();
        }

        boolean completed =
                opModeIsActive() && !anyDriveMotorBusy();

        stopDrive();

        if (!completed && opModeIsActive()) {
            telemetry.addData("Movement", "TIMEOUT");
            telemetry.addData("Stage", stage);
            telemetry.update();
        }

        return completed;
    }

    private boolean anyDriveMotorBusy() {

        return frontLeft.isBusy()
                || frontRight.isBusy()
                || backLeft.isBusy()
                || backRight.isBusy();
    }

    private boolean seesTargetTag() {

        if (limelight == null) {
            return false;
        }

        LLResult result = limelight.getLatestResult();

        if (result == null || !result.isValid()) {
            return false;
        }

        List<LLResultTypes.FiducialResult> fiducials =
                result.getFiducialResults();

        if (fiducials == null) {
            return false;
        }

        for (LLResultTypes.FiducialResult fiducial : fiducials) {

            int detectedId = fiducial.getFiducialId();

            for (int targetId : TARGET_TAGS) {

                if (detectedId == targetId) {
                    return true;
                }
            }
        }

        return false;
    }

    private void waitForTargetTags()
            throws InterruptedException {

        boolean hasSeenTarget = false;

        ElapsedTime timeSinceLastTarget =
                new ElapsedTime();

        intake.setPower(0);

        while (opModeIsActive()) {

            boolean visible = seesTargetTag();

            if (visible) {

                hasSeenTarget = true;
                timeSinceLastTarget.reset();

                intake.setPower(INTAKE_POWER);

            } else {

                intake.setPower(0);

                if (hasSeenTarget
                        && timeSinceLastTarget.seconds()
                        >= TAG_LOSS_GRACE_SECONDS) {

                    break;
                }
            }

            telemetry.addData("Stage", stage);
            telemetry.addData("Tag visible", visible);
            telemetry.addData("Target ever seen", hasSeenTarget);
            telemetry.addData(
                    "Intake",
                    visible ? "RUNNING" : "STOPPED");

            telemetry.update();

            sleep(50);
        }

        intake.setPower(0);
    }

    private void stopDrive() {

        if (frontLeft != null) {
            frontLeft.setPower(0);
            frontLeft.setMode(
                    DcMotor.RunMode.RUN_USING_ENCODER);
        }

        if (frontRight != null) {
            frontRight.setPower(0);
            frontRight.setMode(
                    DcMotor.RunMode.RUN_USING_ENCODER);
        }

        if (backLeft != null) {
            backLeft.setPower(0);
            backLeft.setMode(
                    DcMotor.RunMode.RUN_USING_ENCODER);
        }

        if (backRight != null) {
            backRight.setPower(0);
            backRight.setMode(
                    DcMotor.RunMode.RUN_USING_ENCODER);
        }
    }

    private void stopAll() {

        stopDrive();

        if (intake != null) {
            intake.setPower(0);
        }
    }
}
