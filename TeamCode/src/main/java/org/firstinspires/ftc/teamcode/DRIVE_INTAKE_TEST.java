

package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;

@Autonomous(name = "DRIVE INTAKE TEST", group = "Test")
public class DRIVE_INTAKE_TEST extends LinearOpMode {

    private DcMotor frontLeft;
    private DcMotor frontRight;
    private DcMotor backLeft;
    private DcMotor backRight;

    private Servo leftIntake;
    private Servo rightIntake;

    private static final double WHEEL_DIAMETER = 4.0;
    private static final double TICKS_PER_REV = 537.7;
    private static final double STRAFE_FACTOR = 1.15;

    // Start with short movements for safe testing.
    private static final double TEST_DISTANCE = 12.0;
    private static final double DRIVE_POWER = 0.25;
    private static final double STRAFE_POWER = 0.20;
    private static final double MOVE_TIMEOUT = 6.0;

    // Adjust these positions to match your intake mechanism.
    private static final double INTAKE_OPEN = 0.20;
    private static final double INTAKE_CLOSED = 0.80;

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

        // Change these names if your Robot Configuration differs.
        leftIntake = hardwareMap.get(Servo.class, "left_intake");
        rightIntake = hardwareMap.get(Servo.class, "right_intake");

        frontLeft.setDirection(DcMotorSimple.Direction.REVERSE);
        backLeft.setDirection(DcMotorSimple.Direction.REVERSE);
        frontRight.setDirection(DcMotorSimple.Direction.FORWARD);
        backRight.setDirection(DcMotorSimple.Direction.FORWARD);

        setBrake();
        stopDrive();
        resetEncoders();

        // Start intake in the open position.
        setIntake(INTAKE_OPEN);

        telemetry.addLine("DRIVE INTAKE TEST");
        telemetry.addLine("Movement distance: 12 inches");
        telemetry.addLine("Check clear space around robot.");
        telemetry.addLine("Intake starts OPEN.");
        telemetry.update();

        waitForStart();

        if (!opModeIsActive()) {
            stopDrive();
            return;
        }

        // 1. Backward, then return to starting position.
        if (!drive(-TEST_DISTANCE)) {
            finish();
            return;
        }
        if (!drive(TEST_DISTANCE)) {
            finish();
            return;
        }

        // 2. Forward, then return to starting position.
        if (!drive(TEST_DISTANCE)) {
            finish();
            return;
        }
        if (!drive(-TEST_DISTANCE)) {
            finish();
            return;
        }

        // 3. Strafe left, then return to starting position.
        if (!strafe(-TEST_DISTANCE)) {
            finish();
            return;
        }
        if (!strafe(TEST_DISTANCE)) {
            finish();
            return;
        }

        // 4. Strafe right, then return to starting position.
        if (!strafe(TEST_DISTANCE)) {
            finish();
            return;
        }
        if (!strafe(-TEST_DISTANCE)) {
            finish();
            return;
        }

        stopDrive();

        // 5. Simulate an intake cycle.
        telemetry.addLine("Drive tests complete.");
        telemetry.addLine("Testing intake servos.");
        telemetry.update();

        if (!opModeIsActive()) {
            finish();
            return;
        }

        // Open, pause, close to grab, pause, then open to release.
        setIntake(INTAKE_OPEN);
        sleep(1000);

        if (!opModeIsActive()) {
            finish();
            return;
        }

        setIntake(INTAKE_CLOSED);
        sleep(1500);

        if (!opModeIsActive()) {
            finish();
            return;
        }

        setIntake(INTAKE_OPEN);
        sleep(1000);

        finish();
        telemetry.addLine("ALL TESTS COMPLETE");
        telemetry.update();
    }

    private boolean drive(double inches) {
        int ticks = inchesToTicks(inches);

        setTargets(ticks, ticks, ticks, ticks);

        return runMovement(
                DRIVE_POWER,
                inches < 0 ? "Moving backward" : "Moving forward");
    }

    private boolean strafe(double inches) {
        int ticks = inchesToTicks(inches * STRAFE_FACTOR);

        // Mecanum diagonal pattern.
        setTargets(ticks, -ticks, -ticks, ticks);

        return runMovement(
                STRAFE_POWER,
                inches < 0 ? "Strafing left" : "Strafing right");
    }

    private int inchesToTicks(double inches) {
        double circumference = Math.PI * WHEEL_DIAMETER;

        return (int) Math.round(
                inches / circumference * TICKS_PER_REV);
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
        frontLeft.setPower(power);
        frontRight.setPower(power);
        backLeft.setPower(power);
        backRight.setPower(power);

        ElapsedTime timer = new ElapsedTime();

        while (opModeIsActive()
                && anyMotorBusy()
                && timer.seconds() < MOVE_TIMEOUT) {

            telemetry.addData("Movement", movement);
            telemetry.addData("Elapsed", "%.1f s",
                    timer.seconds());

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
        setDriveMode(DcMotor.RunMode.RUN_USING_ENCODER);

        if (!completed && opModeIsActive()) {
            telemetry.addLine("Movement timed out.");
            telemetry.update();
        }

        return completed;
    }

    private boolean anyMotorBusy() {
        return frontLeft.isBusy()
                || frontRight.isBusy()
                || backLeft.isBusy()
                || backRight.isBusy();
    }

    private void resetEncoders() {
        setDriveMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        setDriveMode(DcMotor.RunMode.RUN_USING_ENCODER);
    }

    private void setDriveMode(DcMotor.RunMode mode) {
        frontLeft.setMode(mode);
        frontRight.setMode(mode);
        backLeft.setMode(mode);
        backRight.setMode(mode);
    }

    private void setBrake() {
        frontLeft.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        frontRight.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        backLeft.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        backRight.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
    }

    private void setIntake(double position) {
        leftIntake.setPosition(position);
        rightIntake.setPosition(position);

        telemetry.addData("Intake position", "%.2f", position);
        telemetry.update();
    }

    private void stopDrive() {
        frontLeft.setPower(0);
        frontRight.setPower(0);
        backLeft.setPower(0);
        backRight.setPower(0);
    }

    private void finish() {
        stopDrive();
        setDriveMode(DcMotor.RunMode.RUN_USING_ENCODER);
    }
}
