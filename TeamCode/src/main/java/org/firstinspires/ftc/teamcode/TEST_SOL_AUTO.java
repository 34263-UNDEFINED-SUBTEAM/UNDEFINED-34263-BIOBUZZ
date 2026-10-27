package org.firstinspires.ftc.teamcode;

import com.qualcomm.hardware.limelightvision.LLResult;
import com.qualcomm.hardware.limelightvision.LLResultTypes;
import com.qualcomm.hardware.limelightvision.Limelight3A;
import com.qualcomm.hardware.sparkfun.SparkFunOTOS;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;

import java.util.List;

@Autonomous(name = "TEST SOL AUTO", group = "Autonomous")
public class TEST_SOL_AUTO extends LinearOpMode {

    private DcMotor frontLeft;
    private DcMotor frontRight;
    private DcMotor backLeft;
    private DcMotor backRight;

    private SparkFunOTOS otos;
    private Limelight3A limelight;

    private static final double SIDE_LENGTH = 48.0;
    private static final double DRIVE_POWER = 0.35;
    private static final double STRAFE_POWER = 0.25;
    private static final double POSITION_TOLERANCE = 1.5;
    private static final double MOVE_TIMEOUT = 20.0;

    @Override
    public void runOpMode() throws InterruptedException {

        // Drive motors: names must match Robot Configuration.
        frontLeft = hardwareMap.get(
                DcMotor.class, "front_left_drive");
        frontRight = hardwareMap.get(
                DcMotor.class, "front_right_drive");
        backLeft = hardwareMap.get(
                DcMotor.class, "back_left_drive");
        backRight = hardwareMap.get(
                DcMotor.class, "back_right_drive");

        frontLeft.setDirection(DcMotor.Direction.REVERSE);
        backLeft.setDirection(DcMotor.Direction.REVERSE);
        frontRight.setDirection(DcMotor.Direction.FORWARD);
        backRight.setDirection(DcMotor.Direction.FORWARD);

        setDriveMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        stopRobot();

        // OTOS: class and hardware name are spelled exactly.
        otos = hardwareMap.get(
                SparkFunOTOS.class, "sensor_otos");

        otos.setLinearUnit(DistanceUnit.INCH);
        otos.setAngularUnit(AngleUnit.DEGREES);

        telemetry.addLine("Keep robot still: calibrating OTOS");
        telemetry.update();

        otos.calibrateImu();
        otos.resetTracking();

        otos.setPosition(new SparkFunOTOS.Pose2D(0, 0, 0));

        telemetry.addLine("OTOS ready");
        telemetry.addLine("Square first, Limelight second");
        telemetry.update();

        waitForStart();

        if (!opModeIsActive()) {
            stopRobot();
            return;
        }

        // ==========================================
        // PHASE 1: 4 FT x 4 FT SQUARE
        // OTOS convention: X = lateral, Y = forward.
        // Assumes zero heading and standard sensor mounting.
        // ==========================================

        // 1. Strafe left 48 inches.
        if (!driveTo(-SIDE_LENGTH, 0)) {
            finishSafely();
            return;
        }

        // 2. Drive forward 48 inches.
        if (!driveTo(-SIDE_LENGTH, SIDE_LENGTH)) {
            finishSafely();
            return;
        }

        // 3. Strafe right 48 inches.
        if (!driveTo(0, SIDE_LENGTH)) {
            finishSafely();
            return;
        }

        // 4. Drive backward 48 inches.
        if (!driveTo(0, 0)) {
            finishSafely();
            return;
        }

        stopRobot();

        // ==========================================
        // PHASE 2: LIMELIGHT APRILTAG STRAFING
        // ==========================================

        telemetry.addLine("Square complete");
        telemetry.addLine("Starting Limelight");
        telemetry.update();

        limelight = hardwareMap.get(
                Limelight3A.class, "limelight");

        limelight.pipelineSwitch(0);
        limelight.start();

        while (opModeIsActive()) {

            LLResult result = limelight.getLatestResult();

            boolean leftFound = false;
            boolean rightFound = false;

            if (result != null && result.isValid()) {

                List<LLResultTypes.FiducialResult> tags =
                        result.getFiducialResults();

                for (LLResultTypes.FiducialResult tag : tags) {

                    int id = tag.getFiducialId();

                    telemetry.addData("AprilTag ID", id);

                    if (id == 43 || id == 44) {
                        leftFound = true;
                    }

                    if (id == 45 || id == 46) {
                        rightFound = true;
                    }
                }
            }

            if (leftFound && rightFound) {

                stopRobot();
                telemetry.addLine("Conflicting tags - STOP");

            } else if (leftFound) {

                // Strafe left.
                frontLeft.setPower(-STRAFE_POWER);
                frontRight.setPower(STRAFE_POWER);
                backLeft.setPower(STRAFE_POWER);
                backRight.setPower(-STRAFE_POWER);

                telemetry.addLine("Tags 43/44: STRAFE LEFT");

            } else if (rightFound) {

                // Strafe right.
                frontLeft.setPower(STRAFE_POWER);
                frontRight.setPower(-STRAFE_POWER);
                backLeft.setPower(-STRAFE_POWER);
                backRight.setPower(STRAFE_POWER);

                telemetry.addLine("Tags 45/46: STRAFE RIGHT");

            } else {

                stopRobot();
                telemetry.addLine("No matching tag - STOP");
            }

            SparkFunOTOS.Pose2D pos = otos.getPosition();

            telemetry.addData("OTOS X", "%.1f", pos.x);
            telemetry.addData("OTOS Y", "%.1f", pos.y);
            telemetry.addData("OTOS Heading", "%.1f", pos.h);

            telemetry.update();
        }

        finishSafely();
    }

    // Drives toward an OTOS X/Y target.
    private boolean driveTo(double targetX, double targetY) {

        ElapsedTime timer = new ElapsedTime();

        while (opModeIsActive()
                && timer.seconds() < MOVE_TIMEOUT) {

            SparkFunOTOS.Pose2D pos = otos.getPosition();

            double errorX = targetX - pos.x;
            double errorY = targetY - pos.y;

            telemetry.addData("Target X", "%.1f", targetX);
            telemetry.addData("Target Y", "%.1f", targetY);
            telemetry.addData("Current X", "%.1f", pos.x);
            telemetry.addData("Current Y", "%.1f", pos.y);

            if (Math.abs(errorX) <= POSITION_TOLERANCE
                    && Math.abs(errorY) <= POSITION_TOLERANCE) {

                stopRobot();
                telemetry.update();
                return true;
            }

            // Positive OTOS X is assumed to mean right.
            // Positive OTOS Y is assumed to mean forward.
            double left = limit(-errorX * 0.035, STRAFE_POWER);
            double forward = limit(errorY * 0.035, DRIVE_POWER);

            if (Math.abs(errorX) > POSITION_TOLERANCE
                    && Math.abs(left) < 0.12) {
                left = Math.signum(-errorX) * 0.12;
            }

            if (Math.abs(errorY) > POSITION_TOLERANCE
                    && Math.abs(forward) < 0.12) {
                forward = Math.signum(errorY) * 0.12;
            }

            // Mecanum power mix.
            double fl = forward - left;
            double fr = forward + left;
            double bl = forward + left;
            double br = forward - left;

            double max = Math.max(1.0, Math.max(
                    Math.max(Math.abs(fl), Math.abs(fr)),
                    Math.max(Math.abs(bl), Math.abs(br))));

            frontLeft.setPower(fl / max);
            frontRight.setPower(fr / max);
            backLeft.setPower(bl / max);
            backRight.setPower(br / max);

            telemetry.update();
        }

        stopRobot();

        telemetry.addLine("Movement timed out - stopping");
        telemetry.update();

        return false;
    }

    private double limit(double value, double max) {
        return Math.max(-max, Math.min(max, value));
    }

    private void setDriveMode(DcMotor.RunMode mode) {
        frontLeft.setMode(mode);
        frontRight.setMode(mode);
        backLeft.setMode(mode);
        backRight.setMode(mode);
    }

    private void stopRobot() {
        frontLeft.setPower(0);
        frontRight.setPower(0);
        backLeft.setPower(0);
        backRight.setPower(0);
    }

    private void finishSafely() {
        stopRobot();

        if (limelight != null) {
            limelight.stop();
        }
    }
}