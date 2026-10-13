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

    private static final double TICKS_PER_REV = 537.7;
    private static final double WHEEL_DIAMETER = 4.0;
    private static final double TICKS_PER_INCH =
            TICKS_PER_REV / (Math.PI * WHEEL_DIAMETER);

    private static final double SQUARE_POWER = 0.4;
    private static final double STRAFE_POWER = 0.25;
    private static final double SIDE_LENGTH = 48.0;
    private static final double MOVE_TIMEOUT = 12.0;

    @Override
    public void runOpMode() throws InterruptedException {

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

        stopRobot();

        setDriveMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        setDriveMode(DcMotor.RunMode.RUN_USING_ENCODER);

        telemetry.addLine("Ready to run square");
        telemetry.update();

        waitForStart();

        if (!opModeIsActive()) {
            return;
        }

        // PHASE 1: COMPLETE THE SQUARE

        telemetry.addLine("Square 1: Strafe Left");
        telemetry.update();

        if (!moveRobot(-SIDE_LENGTH, SIDE_LENGTH,
                SIDE_LENGTH, -SIDE_LENGTH)) {
            stopRobot();
            return;
        }

        telemetry.addLine("Square 2: Forward");
        telemetry.update();

        if (!moveRobot(SIDE_LENGTH, SIDE_LENGTH,
                SIDE_LENGTH, SIDE_LENGTH)) {
            stopRobot();
            return;
        }

        telemetry.addLine("Square 3: Strafe Right");
        telemetry.update();

        if (!moveRobot(SIDE_LENGTH, -SIDE_LENGTH,
                -SIDE_LENGTH, SIDE_LENGTH)) {
            stopRobot();
            return;
        }

        telemetry.addLine("Square 4: Backward");
        telemetry.update();

        if (!moveRobot(-SIDE_LENGTH, -SIDE_LENGTH,
                -SIDE_LENGTH, -SIDE_LENGTH)) {
            stopRobot();
            return;
        }

        stopRobot();

        // PHASE 2: LIMELIGHT APRILTAG STRAFING

        telemetry.addLine("Square complete");
        telemetry.addLine("Initializing Limelight");
        telemetry.update();

        limelight = hardwareMap.get(
                Limelight3A.class, "limelight");

        limelight.pipelineSwitch(0);
        limelight.start();

        sleep(250);

        while (opModeIsActive()) {

            LLResult result = limelight.getLatestResult();

            boolean leftTagFound = false;
            boolean rightTagFound = false;

            if (result != null && result.isValid()) {

                List<LLResultTypes.FiducialResult> tags =
                        result.getFiducialResults();

                for (LLResultTypes.FiducialResult tag : tags) {

                    int id = tag.getFiducialId();

                    telemetry.addData("Tag ID", id);

                    if (id == 43 || id == 44) {
                        leftTagFound = true;
                    }

                    if (id == 45 || id == 46) {
                        rightTagFound = true;
                    }
                }
            }

            if (leftTagFound && rightTagFound) {

                stopRobot();
                telemetry.addLine("Conflicting tags: STOP");

            } else if (leftTagFound) {

                // Strafe left.
                frontLeft.setPower(-STRAFE_POWER);
                frontRight.setPower(STRAFE_POWER);
                backLeft.setPower(STRAFE_POWER);
                backRight.setPower(-STRAFE_POWER);

                telemetry.addLine("Tags 43/44: STRAFE LEFT");

            } else if (rightTagFound) {

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

            telemetry.update();
        }

        stopRobot();
        limelight.stop();
    }

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
        setDrivePower(SQUARE_POWER);

        ElapsedTime timer = new ElapsedTime();

        while (opModeIsActive()
                && timer.seconds() < MOVE_TIMEOUT
                && (frontLeft.isBusy()
                || frontRight.isBusy()
                || backLeft.isBusy()
                || backRight.isBusy())) {

            telemetry.addData("Front Left", "%d / %d",
                    frontLeft.getCurrentPosition(), flTarget);
            telemetry.addData("Front Right", "%d / %d",
                    frontRight.getCurrentPosition(), frTarget);
            telemetry.addData("Back Left", "%d / %d",
                    backLeft.getCurrentPosition(), blTarget);
            telemetry.addData("Back Right", "%d / %d",
                    backRight.getCurrentPosition(), brTarget);

            telemetry.update();
        }

        stopRobot();
        setDriveMode(DcMotor.RunMode.RUN_USING_ENCODER);

        if (!opModeIsActive() || timer.seconds() >= MOVE_TIMEOUT) {
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

    private void setDrivePower(double power) {
        frontLeft.setPower(power);
        frontRight.setPower(power);
        backLeft.setPower(power);
        backRight.setPower(power);
    }

    private void stopRobot() {
        setDrivePower(0);
    }
}