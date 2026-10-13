package org.firstinspires.ftc.teamcode;

import com.qualcomm.hardware.limelightvision.LLResult;
import com.qualcomm.hardware.limelightvision.LLResultTypes;
import com.qualcomm.hardware.limelightvision.Limelight3A;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.util.ElapsedTime;

import java.util.List;

@Autonomous(name = "TEST SOL AUTO", group = "Autonomous")
public class TEST_SOL_AUTO extends LinearOpMode {

    private DcMotor frontLeft;
    private DcMotor frontRight;
    private DcMotor backLeft;
    private DcMotor backRight;

    private Limelight3A limelight;

    // These values must match your actual drivetrain.
    private static final double TICKS_PER_REV = 537.7;
    private static final double WHEEL_DIAMETER = 4.0;
    private static final double TICKS_PER_INCH =
            TICKS_PER_REV / (Math.PI * WHEEL_DIAMETER);

    private static final double DRIVE_POWER = 0.35;
    private static final double STRAFE_POWER = 0.25;
    private static final double SIDE_LENGTH = 48.0;
    private static final double MOVE_TIMEOUT = 12.0;

    @Override
    public void runOpMode() throws InterruptedException {

        // Initialize the four drive motors.
        frontLeft = hardwareMap.get(
                DcMotor.class, "front_left_drive");
        frontRight = hardwareMap.get(
                DcMotor.class, "front_right_drive");
        backLeft = hardwareMap.get(
                DcMotor.class, "back_left_drive");
        backRight = hardwareMap.get(
                DcMotor.class, "back_right_drive");

        // Standard FTC mecanum direction setup.
        frontLeft.setDirection(DcMotor.Direction.REVERSE);
        backLeft.setDirection(DcMotor.Direction.REVERSE);
        frontRight.setDirection(DcMotor.Direction.FORWARD);
        backRight.setDirection(DcMotor.Direction.FORWARD);

        stopRobot();

        setDriveMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        setDriveMode(DcMotor.RunMode.RUN_USING_ENCODER);

        telemetry.addLine("TEST_SOL_AUTO initialized");
        telemetry.addLine("Square first, Limelight second");
        telemetry.update();

        waitForStart();

        if (!opModeIsActive()) {
            return;
        }

        // PHASE 1: DRIVE A 4 FT BY 4 FT SQUARE

        telemetry.addLine("1/4: Strafe left");
        telemetry.update();

        if (!moveRobot(-SIDE_LENGTH, SIDE_LENGTH,
                SIDE_LENGTH, -SIDE_LENGTH)) {
            stopRobot();
            return;
        }

        telemetry.addLine("2/4: Drive forward");
        telemetry.update();

        if (!moveRobot(SIDE_LENGTH, SIDE_LENGTH,
                SIDE_LENGTH, SIDE_LENGTH)) {
            stopRobot();
            return;
        }

        telemetry.addLine("3/4: Strafe right");
        telemetry.update();

        if (!moveRobot(SIDE_LENGTH, -SIDE_LENGTH,
                -SIDE_LENGTH, SIDE_LENGTH)) {
            stopRobot();
            return;
        }

        telemetry.addLine("4/4: Drive backward");
        telemetry.update();

        if (!moveRobot(-SIDE_LENGTH, -SIDE_LENGTH,
                -SIDE_LENGTH, -SIDE_LENGTH)) {
            stopRobot();
            return;
        }

        stopRobot();

        // PHASE 2: LIMELIGHT APRILTAG STRAFING

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
                telemetry.addLine("Conflicting tags - stopped");

            } else if (leftFound) {

                // Strafe left.
                frontLeft.setPower(-STRAFE_POWER);
                frontRight.setPower(STRAFE_POWER);
                backLeft.setPower(STRAFE_POWER);
                backRight.setPower(-STRAFE_POWER);

                telemetry.addLine("Tag 43/44 - STRAFE LEFT");

            } else if (rightFound) {

                // Strafe right.
                frontLeft.setPower(STRAFE_POWER);
                frontRight.setPower(-STRAFE_POWER);
                backLeft.setPower(-STRAFE_POWER);
                backRight.setPower(STRAFE_POWER);

                telemetry.addLine("Tag 45/46 - STRAFE RIGHT");

            } else {

                stopRobot();
                telemetry.addLine("No target tag - stopped");
            }

            telemetry.update();
        }

        stopRobot();
        limelight.stop();
    }

    // Move the wheels by relative encoder distances.
    private boolean moveRobot(
            double flInches,
            double frInches,
            double blInches,
            double brInches) {

        int flTarget = frontLeft.getCurrentPosition()
                + (int) (flInches * TICKS_PER_INCH);

        int frTarget = frontRight.getCurrentPosition()
                + (int) (frInches * TICKS_PER_INCH);

        int blTarget = backLeft.getCurrentPosition()
                + (int) (blInches * TICKS_PER_INCH);

        int brTarget = backRight.getCurrentPosition()
                + (int) (brInches * TICKS_PER_INCH);

        frontLeft.setTargetPosition(flTarget);
        frontRight.setTargetPosition(frTarget);
        backLeft.setTargetPosition(blTarget);
        backRight.setTargetPosition(brTarget);

        setDriveMode(DcMotor.RunMode.RUN_TO_POSITION);

        frontLeft.setPower(DRIVE_POWER);
        frontRight.setPower(DRIVE_POWER);
        backLeft.setPower(DRIVE_POWER);
        backRight.setPower(DRIVE_POWER);

        ElapsedTime timer = new ElapsedTime();

        while (opModeIsActive()
                && timer.seconds() < MOVE_TIMEOUT
                && (frontLeft.isBusy()
                || frontRight.isBusy()
                || backLeft.isBusy()
                || backRight.isBusy())) {

            telemetry.addData("Front Left",
                    "%d / %d",
                    frontLeft.getCurrentPosition(), flTarget);

            telemetry.addData("Front Right",
                    "%d / %d",
                    frontRight.getCurrentPosition(), frTarget);

            telemetry.addData("Back Left",
                    "%d / %d",
                    backLeft.getCurrentPosition(), blTarget);

            telemetry.addData("Back Right",
                    "%d / %d",
                    backRight.getCurrentPosition(), brTarget);

            telemetry.update();
        }

        stopRobot();
        setDriveMode(DcMotor.RunMode.RUN_USING_ENCODER);

        if (!opModeIsActive()
                || timer.seconds() >= MOVE_TIMEOUT) {

            telemetry.addLine(
                    "Movement stopped or timed out");

            telemetry.update();
            return false;
        }

        return true;
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
}