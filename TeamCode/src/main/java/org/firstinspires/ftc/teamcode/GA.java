
package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.util.ElapsedTime;

@Autonomous(name = "GA - Garden Autonomous", group = "Competition")
public class GA extends LinearOpMode {

    private DcMotor frontLeft;
    private DcMotor frontRight;
    private DcMotor backLeft;
    private DcMotor backRight;

    private final ElapsedTime matchTimer = new ElapsedTime();

    // CALIBRATE THESE BEFORE ENABLING MOVEMENT.
    private static final double TICKS_PER_INCH = 0.0;
    private static final double TRACK_WIDTH_INCHES = 0.0;

    private static final double AUTO_TIME_LIMIT = 29.0;

    @Override
    public void runOpMode() {

        initializeRobot();

        telemetry.addLine("GA - Garden Autonomous");
        telemetry.addLine("Four-motor mecanum drive");
        telemetry.addLine("Checking encoder calibration...");

        if (TICKS_PER_INCH <= 0 || TRACK_WIDTH_INCHES <= 0) {
            telemetry.addLine("CALIBRATION REQUIRED.");
            telemetry.addLine("Movement is disabled until calibrated.");
            telemetry.update();
            return;
        }

        telemetry.addLine("Ready. Waiting for start.");
        telemetry.update();

        waitForStart();

        if (isStopRequested()) {
            stopDrive();
            return;
        }

        matchTimer.reset();

        try {

            shootPlaceholder();

            if (!timeAvailable()) return;
            drive(28, 0.5);

            if (!timeAvailable()) return;
            turn(-35, 0.4);

            if (!timeAvailable()) return;
            drive(20, 0.5);

            if (!timeAvailable()) return;
            collectPlaceholder(2.5);

            if (!timeAvailable()) return;
            shootPlaceholder();

            if (!timeAvailable()) return;
            pollenReleasePlaceholder();

            if (!timeAvailable()) return;
            turn(70, 0.4);

            if (!timeAvailable()) return;
            drive(22, 0.5);

            if (!timeAvailable()) return;
            collectPlaceholder(3.0);

        } finally {
            stopDrive();
        }

        telemetry.addLine("GA finished.");
        telemetry.update();
    }

    private void initializeRobot() {

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

        setBrakeBehavior();

        resetEncoders();

        setRunUsingEncoders();

        stopDrive();
    }

    private void setBrakeBehavior() {

        frontLeft.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        frontRight.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        backLeft.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        backRight.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
    }

    private void resetEncoders() {

        frontLeft.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        frontRight.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        backLeft.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        backRight.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
    }

    private void setRunUsingEncoders() {

        frontLeft.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        frontRight.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        backLeft.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        backRight.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
    }

    private boolean timeAvailable() {

        if (isStopRequested()) {
            return false;
        }

        if (matchTimer.seconds() >= AUTO_TIME_LIMIT) {
            telemetry.addLine("Autonomous time limit reached.");
            telemetry.update();
            stopDrive();
            return false;
        }

        return true;
    }

    private void drive(double inches, double power) {

        int ticks = (int) (inches * TICKS_PER_INCH);

        moveMecanum(
                ticks, ticks,
                ticks, ticks,
                power
        );

        telemetry.addData("Drive", "%.1f inches", inches);
        telemetry.update();
    }

    private void turn(double degrees, double power) {

        double arcDistance =
                Math.PI * TRACK_WIDTH_INCHES * degrees / 360.0;

        int ticks = (int) (arcDistance * TICKS_PER_INCH);

        moveMecanum(
                ticks, -ticks,
                ticks, -ticks,
                power
        );

        telemetry.addData("Turn", "%.1f degrees", degrees);
        telemetry.update();
    }

    private void moveMecanum(
            int flTicks,
            int frTicks,
            int blTicks,
            int brTicks,
            double power) {

        if (!opModeIsActive() || !timeAvailable()) {
            stopDrive();
            return;
        }

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

        ElapsedTime movementTimer = new ElapsedTime();

        while (opModeIsActive()
                && timeAvailable()
                && (frontLeft.isBusy()
                || frontRight.isBusy()
                || backLeft.isBusy()
                || backRight.isBusy())
                && movementTimer.seconds() < 8.0) {

            telemetry.addData("FL", frontLeft.getCurrentPosition());
            telemetry.addData("FR", frontRight.getCurrentPosition());
            telemetry.addData("BL", backLeft.getCurrentPosition());
            telemetry.addData("BR", backRight.getCurrentPosition());
            telemetry.update();

            idle();
        }

        stopDrive();
        setRunUsingEncoders();
    }

    private void collectPlaceholder(double seconds) {
        telemetry.addData("Collect", "Not configured yet (%.1f sec)", seconds);
        telemetry.update();
    }

    private void shootPlaceholder() {
        telemetry.addLine("Shooting mechanism not configured.");
        telemetry.update();
    }

    private void pollenReleasePlaceholder() {
        telemetry.addLine("Pollen release not configured.");
        telemetry.update();
    }

    private void stopDrive() {

        if (frontLeft != null) frontLeft.setPower(0);
        if (frontRight != null) frontRight.setPower(0);
        if (backLeft != null) backLeft.setPower(0);
        if (backRight != null) backRight.setPower(0);
    }
}
