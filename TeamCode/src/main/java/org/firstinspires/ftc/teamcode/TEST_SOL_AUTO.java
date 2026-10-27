
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

        // Initialize drive motors.
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

        // Find the OTOS by device type, regardless of its configured name.
        List<SparkFunOTOS> otosDevices =
                hardwareMap.getAll(SparkFunOTOS.class);

        if (otosDevices.isEmpty()) {
            telemetry.addLine("ERROR: No SparkFun OTOS found.");
            telemetry.addLine("Check the OTOS device type in Robot Configuration.");
            telemetry.update();
            return;
        }

        if (otosDevices.size() > 1) {
            telemetry.addLine("ERROR: Multiple OTOS devices found.");
            telemetry.addLine("This program expects one OTOS.");
            telemetry.update();
            return;
        }

        otos = otosDevices.get(0);

        // Configure OTOS units and reset its starting position.
        otos.setLinearUnit(DistanceUnit.INCH);
        otos.setAngularUnit(AngleUnit.DEGREES);

        telemetry.addLine("Keep robot still while OTOS calibrates.");
        telemetry.update();

        otos.calibrateImu();
        otos.resetTracking();
        otos.setPosition(new SparkFunOTOS.Pose2D(0, 0, 0));

        // Initialize Limelight before starting the OpMode.
        limelight = hardwareMap.get(
                Limelight3A.class, "limelight");

        limelight.pipelineSwitch(0);

        telemetry.addLine("OTOS ready.");
        telemetry.addLine("Limelight ready.");
        telemetry.addLine("Ready to start.");
        telemetry.update();

        waitForStart();

        if (!opModeIsActive()) {
            finishSafely();
            return;
        }

        // PHASE 1: Drive a 48-inch square.
        // Assumes OTOS X is lateral and Y is forward.
        // Check the direction readings before a full-speed run.

        if (!driveTo(-SIDE_LENGTH, 0)) {
            finishSafely();
            return;
        }

        if (!driveTo(-SIDE_LENGTH, SIDE_LENGTH)) {
            finishSafely();
            return;
        }

        if (!driveTo(0, SIDE_LENGTH)) {
            finishSafely();
            return;
        }

        if (!driveTo(0, 0)) {
            finishSafely();
            return;
        }

        stopRobot();

        // PHASE 2: Detect AprilTags and strafe.
        limelight.start();

        telemetry.addLine("Square complete.");
        telemetry.addLine("Limelight tracking started.");
        telemetry.update();

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
                telemetry.addLine("Conflicting tags: STOP");

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
                telemetry.addLine("No matching tag: STOP");
            }

            SparkFunOTOS.Pose2D pos = otos.getPosition();

            telemetry.addData("OTOS X", "%.1f", pos.x);
            telemetry.addData("OTOS Y", "%.1f", pos.y);
            telemetry.addData("OTOS Heading", "%.1f", pos.h);

            telemetry.update();
            idle();
        }

        finishSafely();
    }

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

            double strafe = limit(
                    -errorX * 0.035, STRAFE_POWER);

            double forward = limit(
                    errorY * 0.035, DRIVE_POWER);

            if (Math.abs(errorX) > POSITION_TOLERANCE
                    && Math.abs(strafe) < 0.12) {
                strafe = Math.signum(-errorX) * 0.12;
            }

            if (Math.abs(errorY) > POSITION_TOLERANCE
                    && Math.abs(forward) < 0.12) {
                forward = Math.signum(errorY) * 0.12;
            }

            // Mecanum drive mixing.
            double fl = forward - strafe;
            double fr = forward + strafe;
            double bl = forward + strafe;
            double br = forward - strafe;

            double max = Math.max(1.0, Math.max(
                    Math.max(Math.abs(fl), Math.abs(fr)),
                    Math.max(Math.abs(bl), Math.abs(br))));

            frontLeft.setPower(fl / max);
            frontRight.setPower(fr / max);
            backLeft.setPower(bl / max);
            backRight.setPower(br / max);

            telemetry.update();
            idle();
        }

        stopRobot();

        telemetry.addLine("Movement timed out. Robot stopped.");
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
