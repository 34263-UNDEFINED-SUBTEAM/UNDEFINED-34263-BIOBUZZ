
package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.util.ElapsedTime;

@Autonomous(name = "TEST SOL AUTO", group = "Autonomous")
public class TEST_SOL_AUTO extends LinearOpMode {

    private DcMotor frontLeft;
    private DcMotor frontRight;
    private DcMotor backLeft;
    private DcMotor backRight;

    private static final double WHEEL_DIAMETER = 4.0;
    private static final double TICKS_PER_REV = 537.7;
    private static final double GEAR_RATIO = 1.0;

    private static final double SIDE_INCHES = 48.0;

    // Keep power low while testing.
    private static final double DRIVE_POWER = 0.30;
    private static final double STRAFE_POWER = 0.22;

    // Calibrate this after testing the actual lateral distance.
    private static final double STRAFE_FACTOR = 1.15;

    private static final double MOVE_TIMEOUT = 10.0;

    @Override
    public void runOpMode() {

        frontLeft = hardwareMap.get(
                DcMotor.class, "front_left_drive");
        frontRight = hardwareMap.get(
                DcMotor.class, "front_right_drive");
        backLeft = hardwareMap.get(
                DcMotor.class, "back_left_drive");
        backRight = hardwareMap.get(
                DcMotor.class, "back_right_drive");

        // Standard mecanum direction setup.
        frontLeft.setDirection(DcMotorSimple.Direction.REVERSE);
        backLeft.setDirection(DcMotorSimple.Direction.REVERSE);
        frontRight.setDirection(DcMotorSimple.Direction.FORWARD);
        backRight.setDirection(DcMotorSimple.Direction.FORWARD);

        setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        stopDrive();

        resetEncoders();

        telemetry.addLine("TEST SOL AUTO");
        telemetry.addLine("Encoder-based 4 ft square");
        telemetry.addLine("Check motor directions before starting.");
        telemetry.update();

        waitForStart();

        if (!opModeIsActive()) {
            stopDrive();
            return;
        }

        // 1. Forward 4 feet.
        if (!driveStraight(SIDE_INCHES)) {
            finish();
            return;
        }

        // 2. Strafe right 4 feet.
        if (!strafe(SIDE_INCHES)) {
            finish();
            return;
        }

        // 3. Backward 4 feet.
        if (!driveStraight(-SIDE_INCHES)) {
            finish();
            return;
        }

        // 4. Strafe left 4 feet to the starting point.
        if (!strafe(-SIDE_INCHES)) {
            finish();
            return;
        }

        finish();

        telemetry.addLine("SQUARE COMPLETE");
        telemetry.update();
    }

    private boolean driveStraight(double inches) {

        int ticks = inchesToTicks(inches);

        // All four wheels move the same distance.
        setTargets(ticks, ticks, ticks, ticks);

        return runMovement(
                DRIVE_POWER,
                inches > 0 ? "Forward" : "Backward");
    }

    private boolean strafe(double inches) {

        int ticks = inchesToTicks(inches * STRAFE_FACTOR);

        /*
         * Mecanum diagonal pattern:
         *
         * Front Left:  +
         * Front Right: -
         * Back Left:   -
         * Back Right:  +
         *
         * Reverse every sign to strafe the other way.
         */
        setTargets(ticks, -ticks, -ticks, ticks);

        return runMovement(
                STRAFE_POWER,
                inches > 0 ? "Strafing right" : "Strafing left");
    }

    private int inchesToTicks(double inches) {

        double circumference = Math.PI * WHEEL_DIAMETER;

        return (int) Math.round(
                inches / circumference
                        * TICKS_PER_REV
                        * GEAR_RATIO);
    }

    private void setTargets(
            int fl, int fr, int bl, int br) {

        frontLeft.setTargetPosition(
                frontLeft.getCurrentPosition() + fl);
        frontRight.setTargetPosition(
                frontRight.getCurrentPosition() + fr);
        backLeft.setTargetPosition(
                backLeft.getCurrentPosition() + bl);
        backRight.setTargetPosition(
                backRight.getCurrentPosition() + br);

        frontLeft.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        frontRight.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        backLeft.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        backRight.setMode(DcMotor.RunMode.RUN_TO_POSITION);
    }

    private boolean runMovement(double power, String movement) {

        // Every motor receives the same power magnitude.
        frontLeft.setPower(power);
        frontRight.setPower(power);
        backLeft.setPower(power);
        backRight.setPower(power);

        ElapsedTime timer = new ElapsedTime();

        while (opModeIsActive()
                && anyMotorBusy()
                && timer.seconds() < MOVE_TIMEOUT) {

            telemetry.addData("Movement", movement);
            telemetry.addData("Time", "%.1f s", timer.seconds());

            telemetry.addData("Front Left", "%d / %d",
                    frontLeft.getCurrentPosition(),
                    frontLeft.getTargetPosition());

            telemetry.addData("Front Right", "%d / %d",
                    frontRight.getCurrentPosition(),
                    frontRight.getTargetPosition());

            telemetry.addData("Back Left", "%d / %d",
                    backLeft.getCurrentPosition(),
                    backLeft.getTargetPosition());

            telemetry.addData("Back Right", "%d / %d",
                    backRight.getCurrentPosition(),
                    backRight.getTargetPosition());

            telemetry.update();
            idle();
        }

        boolean completed = opModeIsActive() && !anyMotorBusy();

        stopDrive();

        if (!completed && opModeIsActive()) {
            telemetry.addLine("TIMEOUT: Stopping movement.");
            telemetry.update();
        }

        setDriveMode(DcMotor.RunMode.RUN_USING_ENCODER);

        return completed;
    }

    private boolean anyMotorBusy() {
        return frontLeft.isBusy()
                || frontRight.isBusy()
                || backLeft.isBusy()
                || backRight.isBusy();
    }

    private void resetEncoders() {

        setDrivePower(0);

        setDriveMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        setDriveMode(DcMotor.RunMode.RUN_USING_ENCODER);
    }

    private void setDriveMode(DcMotor.RunMode mode) {
        frontLeft.setMode(mode);
        frontRight.setMode(mode);
        backLeft.setMode(mode);
        backRight.setMode(mode);
    }

    private void setZeroPowerBehavior(
            DcMotor.ZeroPowerBehavior behavior) {

        frontLeft.setZeroPowerBehavior(behavior);
        frontRight.setZeroPowerBehavior(behavior);
        backLeft.setZeroPowerBehavior(behavior);
        backRight.setZeroPowerBehavior(behavior);
    }

    private void setDrivePower(double power) {
        frontLeft.setPower(power);
        frontRight.setPower(power);
        backLeft.setPower(power);
        backRight.setPower(power);
    }

    private void stopDrive() {
        setDrivePower(0);
    }

    private void finish() {
        stopDrive();
        setDriveMode(DcMotor.RunMode.RUN_USING_ENCODER);
    }
}
