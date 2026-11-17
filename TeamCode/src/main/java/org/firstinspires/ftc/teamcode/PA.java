
package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.util.ElapsedTime;

@Autonomous(name = "PA - Pollen Auto", group = "Autonomous")
public class PA extends LinearOpMode {

    private DcMotor frontLeft;
    private DcMotor frontRight;
    private DcMotor backLeft;
    private DcMotor backRight;

    // Robot configuration
    private static final double TICKS_PER_INCH = 41.81;
    private static final double TURN_DIAMETER_INCHES = 28.0;

    private static final double DRIVE_POWER = 0.55;
    private static final double TURN_POWER = 0.45;

    private static final double MATCH_LIMIT_SECONDS = 29.0;

    private final ElapsedTime matchTimer = new ElapsedTime();

    @Override
    public void runOpMode() {

        initializeHardware();

        telemetry.addLine("PA - Pollen Auto");
        telemetry.addLine("Hardware initialized");
        telemetry.addData("Ticks per inch", TICKS_PER_INCH);
        telemetry.addData("Turn diameter", TURN_DIAMETER_INCHES);
        telemetry.addLine("Ready to start");
        telemetry.update();

        waitForStart();

        if (isStopRequested()) {
            stopDrive();
            return;
        }

        matchTimer.reset();

        telemetry.addLine("Pollen Auto started");
        telemetry.update();

        boolean completed = runPollenRoute();

        stopDrive();

        if (completed) {
            telemetry.addLine("Pollen Auto completed");
        } else {
            telemetry.addLine("Pollen Auto stopped early");
            telemetry.addData("Elapsed time", matchTimer.seconds());
        }

        telemetry.update();

        sleep(1000);
    }

    private void initializeHardware() {

        frontLeft = hardwareMap.get(DcMotor.class, "front_left_drive");
        frontRight = hardwareMap.get(DcMotor.class, "front_right_drive");
        backLeft = hardwareMap.get(DcMotor.class, "back_left_drive");
        backRight = hardwareMap.get(DcMotor.class, "back_right_drive");

        // Tested motor directions
        frontLeft.setDirection(DcMotor.Direction.REVERSE);
        backLeft.setDirection(DcMotor.Direction.REVERSE);

        frontRight.setDirection(DcMotor.Direction.FORWARD);
        backRight.setDirection(DcMotor.Direction.FORWARD);

        DcMotor[] motors = {
                frontLeft, frontRight, backLeft, backRight
        };

        for (DcMotor motor : motors) {
            motor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
            motor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            motor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            motor.setPower(0);
        }
    }

    // Pollen Auto route
    private boolean runPollenRoute() {

        // 1. Forward 4 feet
        if (!drive(48, 0)) return false;

        // 2. Turn 90 degrees left
        if (!turn(-90)) return false;

        // 3. Wait 3 seconds
        if (!waitSafely(3000)) return false;

        // 4. Turn another 110 degrees left
        if (!turn(-110)) return false;

        // 5. Forward 6.5 feet
        if (!drive(78, 0)) return false;

        // 6. Turn 180 degrees right
        if (!turn(180)) return false;

        // 7. Forward 6.5 feet
        if (!drive(78, 0)) return false;

        // 8. Back up 1 foot
        if (!drive(-12, 0)) return false;

        // 9. Turn 125 degrees right
        if (!turn(125)) return false;

        // 10. Forward 6 feet
        if (!drive(72, 0)) return false;

        return true;
    }

    // Encoder-based mecanum movement
    // Positive forward = forward
    // Positive strafeRight = right
    private boolean drive(double forwardInches, double strafeRightInches) {

        if (!canContinue()) return false;

        int flTicks = (int) Math.round(
                (forwardInches + strafeRightInches) * TICKS_PER_INCH);

        int frTicks = (int) Math.round(
                (forwardInches - strafeRightInches) * TICKS_PER_INCH);

        int blTicks = (int) Math.round(
                (forwardInches - strafeRightInches) * TICKS_PER_INCH);

        int brTicks = (int) Math.round(
                (forwardInches + strafeRightInches) * TICKS_PER_INCH);

        return runToPosition(
                flTicks,
                frTicks,
                blTicks,
                brTicks,
                DRIVE_POWER,
                Math.abs(forwardInches) + Math.abs(strafeRightInches)
        );
    }

    // Positive degrees = right/clockwise
    // Negative degrees = left/counterclockwise
    private boolean turn(double degrees) {

        if (!canContinue()) return false;

        double wheelDistance =
                Math.PI * TURN_DIAMETER_INCHES * Math.abs(degrees) / 360.0;

        int ticks = (int) Math.round(
                wheelDistance * TICKS_PER_INCH);

        int direction = degrees >= 0 ? 1 : -1;

        int flTicks = direction * ticks;
        int frTicks = -direction * ticks;
        int blTicks = direction * ticks;
        int brTicks = -direction * ticks;

        return runToPosition(
                flTicks,
                frTicks,
                blTicks,
                brTicks,
                TURN_POWER,
                wheelDistance
        );
    }

    private boolean runToPosition(
            int flTicks,
            int frTicks,
            int blTicks,
            int brTicks,
            double power,
            double distanceInches) {

        if (!canContinue()) return false;

        DcMotor[] motors = {
                frontLeft, frontRight, backLeft, backRight
        };

        int[] targets = {
                flTicks, frTicks, blTicks, brTicks
        };

        for (int i = 0; i < motors.length; i++) {
            motors[i].setTargetPosition(
                    motors[i].getCurrentPosition() + targets[i]);
            motors[i].setMode(DcMotor.RunMode.RUN_TO_POSITION);
        }

        for (int i = 0; i < motors.length; i++) {
            motors[i].setPower(power);
        }

        double timeoutSeconds = 2.0 + distanceInches / 18.0;
        ElapsedTime movementTimer = new ElapsedTime();
        movementTimer.reset();

        boolean timedOut = false;

        while (opModeIsActive()
                && canContinue()
                && anyMotorBusy()
                && movementTimer.seconds() < timeoutSeconds) {

            telemetry.addData("Movement time", "%.1f", movementTimer.seconds());
            telemetry.addData("Match time", "%.1f", matchTimer.seconds());

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

        if (movementTimer.seconds() >= timeoutSeconds
                && anyMotorBusy()) {
            timedOut = true;
        }

        stopDrive();

        for (DcMotor motor : motors) {
            motor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        }

        return !timedOut && canContinue();
    }

    private boolean anyMotorBusy() {
        return frontLeft.isBusy()
                || frontRight.isBusy()
                || backLeft.isBusy()
                || backRight.isBusy();
    }

    private boolean waitSafely(long milliseconds) {

        ElapsedTime waitTimer = new ElapsedTime();
        waitTimer.reset();

        while (opModeIsActive()
                && canContinue()
                && waitTimer.milliseconds() < milliseconds) {

            telemetry.addData("Waiting", "%.1f seconds",
                    waitTimer.seconds());
            telemetry.update();

            idle();
        }

        return canContinue();
    }

    private boolean canContinue() {
        return opModeIsActive()
                && matchTimer.seconds() < MATCH_LIMIT_SECONDS;
    }

    private void stopDrive() {

        if (frontLeft != null) frontLeft.setPower(0);
        if (frontRight != null) frontRight.setPower(0);
        if (backLeft != null) backLeft.setPower(0);
        if (backRight != null) backRight.setPower(0);
    }
}
