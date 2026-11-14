
package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;

@Autonomous(name = "GA - Garden Autonomous", group = "Competition")
public class GA extends LinearOpMode {

    // Hardware
    private DcMotor frontLeft;
    private DcMotor frontRight;
    private DcMotor backLeft;
    private DcMotor backRight;

    // Initial encoder calibration
    private static final double TICKS_PER_INCH = 41.81;

    // Conservative starting power
    private static final double DRIVE_POWER = 0.30;
    private static final double TURN_POWER = 0.25;

    private static final double MATCH_TIME_SECONDS = 29.0;

    private long matchStartTime;

    @Override
    public void runOpMode() {

        // Map all four mecanum drive motors
        frontLeft = hardwareMap.get(
                DcMotor.class, "front_left_drive");

        frontRight = hardwareMap.get(
                DcMotor.class, "front_right_drive");

        backLeft = hardwareMap.get(
                DcMotor.class, "back_left_drive");

        backRight = hardwareMap.get(
                DcMotor.class, "back_right_drive");

        // Initial direction assumption.
        // Verify with wheels lifted before driving.
        frontLeft.setDirection(DcMotor.Direction.REVERSE);
        backLeft.setDirection(DcMotor.Direction.REVERSE);

        frontRight.setDirection(DcMotor.Direction.FORWARD);
        backRight.setDirection(DcMotor.Direction.FORWARD);

        resetDriveEncoders();

        telemetry.addLine("GA - GARDEN AUTONOMOUS");
        telemetry.addLine("Four-motor mecanum drive");
        telemetry.addData("Ticks per inch", TICKS_PER_INCH);
        telemetry.addLine("Initial calibration - TEST REQUIRED");
        telemetry.addLine("Press START when ready.");
        telemetry.update();

        waitForStart();

        if (isStopRequested()) {
            stopRobot();
            return;
        }

        matchStartTime = System.nanoTime();

        // ==================================
        // GARDEN AUTONOMOUS ROUTE
        // ==================================

        // 1. Initial shooting position
        shootPlaceholder();

        // 2. Move forward 28 inches
        drive(28);

        // 3. Turn approximately -35 degrees
        turnPlaceholder(-35);

        // 4. Move forward 20 inches
        drive(20);

        // 5. Collect
        collectPlaceholder(2.5);

        // 6. Shoot
        shootPlaceholder();

        // 7. Release pollen
        pollenReleasePlaceholder();

        // 8. Turn approximately 70 degrees
        turnPlaceholder(70);

        // 9. Move forward 22 inches
        drive(22);

        // 10. Collect again
        collectPlaceholder(3.0);

        stopRobot();

        telemetry.addLine("AUTONOMOUS COMPLETE");
        telemetry.update();
    }

    // ==================================
    // DRIVE CONTROL
    // ==================================

    private void drive(double inches) {

        if (!opModeIsActive() || timeExpired()) {
            return;
        }

        int ticks = (int) Math.round(inches * TICKS_PER_INCH);

        setTarget(
                frontLeft.getCurrentPosition() + ticks,
                frontRight.getCurrentPosition() + ticks,
                backLeft.getCurrentPosition() + ticks,
                backRight.getCurrentPosition() + ticks
        );

        setMode(DcMotor.RunMode.RUN_TO_POSITION);
        setPower(DRIVE_POWER);

        waitForMotors();

        stopDrive();
        setMode(DcMotor.RunMode.RUN_USING_ENCODER);
    }

    // ==================================
    // TURNING
    // ==================================

    private void turnPlaceholder(double degrees) {

        telemetry.addData(
                "TURN NOT CALIBRATED",
                "%.1f degrees", degrees
        );

        telemetry.addLine(
                "Turn skipped for safety."
        );

        telemetry.update();

        // No guessed turn distance or track width.
        sleep(300);
    }

    // ==================================
    // PLACEHOLDER MECHANISMS
    // ==================================

    private void collectPlaceholder(double seconds) {

        telemetry.addData("COLLECT", "Placeholder: %.1f seconds", seconds);
        telemetry.update();

        sleep((long) (seconds * 1000));
    }

    private void shootPlaceholder() {

        telemetry.addLine("SHOOT: Mechanism not configured");
        telemetry.update();

        sleep(300);
    }

    private void pollenReleasePlaceholder() {

        telemetry.addLine("POLLEN RELEASE: Not configured");
        telemetry.update();

        sleep(300);
    }

    // ==================================
    // ENCODER HELPERS
    // ==================================

    private void resetDriveEncoders() {

        setPower(0);

        setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        setMode(DcMotor.RunMode.RUN_USING_ENCODER);
    }

    private void setTarget(
            int fl, int fr, int bl, int br) {

        frontLeft.setTargetPosition(fl);
        frontRight.setTargetPosition(fr);
        backLeft.setTargetPosition(bl);
        backRight.setTargetPosition(br);
    }

    private void setMode(DcMotor.RunMode mode) {

        frontLeft.setMode(mode);
        frontRight.setMode(mode);
        backLeft.setMode(mode);
        backRight.setMode(mode);
    }

    private void setPower(double power) {

        frontLeft.setPower(power);
        frontRight.setPower(power);
        backLeft.setPower(power);
        backRight.setPower(power);
    }

    private void waitForMotors() {

        long timeout = System.currentTimeMillis() + 8000;

        while (opModeIsActive()
                && !timeExpired()
                && System.currentTimeMillis() < timeout
                && (frontLeft.isBusy()
                || frontRight.isBusy()
                || backLeft.isBusy()
                || backRight.isBusy())) {

            telemetry.addData("FL", frontLeft.getCurrentPosition());
            telemetry.addData("FR", frontRight.getCurrentPosition());
            telemetry.addData("BL", backLeft.getCurrentPosition());
            telemetry.addData("BR", backRight.getCurrentPosition());
            telemetry.update();

            idle();
        }
    }

    // ==================================
    // SAFETY
    // ==================================

    private boolean timeExpired() {

        double elapsed =
                (System.nanoTime() - matchStartTime) / 1_000_000_000.0;

        return elapsed >= MATCH_TIME_SECONDS;
    }

    private void stopDrive() {
        setPower(0);
    }

    private void stopRobot() {

        if (frontLeft != null) frontLeft.setPower(0);
        if (frontRight != null) frontRight.setPower(0);
        if (backLeft != null) backLeft.setPower(0);
        if (backRight != null) backRight.setPower(0);
    }
}
