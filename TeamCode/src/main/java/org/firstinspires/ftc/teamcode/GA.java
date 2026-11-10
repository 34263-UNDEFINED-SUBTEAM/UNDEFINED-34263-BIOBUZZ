
package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.util.ElapsedTime;

@Autonomous(name = "GA - Garden Autonomous", group = "Competition")
public class GA extends LinearOpMode {

    // Drive motors
    private DcMotor leftDrive;
    private DcMotor rightDrive;

    // Intake hardware
    private DcMotor intake;
    private CRServo leftIntakeServo;
    private CRServo rightIntakeServo;

    // Match timer
    private final ElapsedTime matchTimer = new ElapsedTime();

    // Starting estimates - must be calibrated
    private static final double TICKS_PER_INCH = 45.0;
    private static final double TRACK_WIDTH = 14.0;
    private static final double AUTO_TIME_LIMIT = 29.0;

    @Override
    public void runOpMode() {

        initializeRobot();

        telemetry.addLine("GA - Garden Autonomous");
        telemetry.addLine("Robot initialized.");
        telemetry.addLine("Waiting for start...");
        telemetry.update();

        waitForStart();

        if (isStopRequested()) {
            stopAll();
            return;
        }

        matchTimer.reset();

        try {

            // Initial scoring placeholder
            shoot();

            // Move toward first collection area
            if (!timeAvailable()) return;
            drive(28, 0.5);

            if (!timeAvailable()) return;
            turn(-35, 0.4);

            if (!timeAvailable()) return;
            drive(20, 0.5);

            // Collect
            if (!timeAvailable()) return;
            collect(2.5);

            // Second scoring placeholder
            if (!timeAvailable()) return;
            shoot();

            // Pollen release placeholder
            if (!timeAvailable()) return;
            pollenReleasePlaceholder();

            // Move toward next collection area
            if (!timeAvailable()) return;
            turn(70, 0.4);

            if (!timeAvailable()) return;
            drive(22, 0.5);

            if (!timeAvailable()) return;
            collect(3.0);

        } finally {
            stopAll();
        }

        telemetry.addLine("GA Autonomous finished.");
        telemetry.update();
    }

    private void initializeRobot() {

        leftDrive = hardwareMap.get(DcMotor.class, "left_drive");
        rightDrive = hardwareMap.get(DcMotor.class, "right_drive");

        intake = hardwareMap.get(DcMotor.class, "intake");

        leftIntakeServo =
                hardwareMap.get(CRServo.class, "left_intake_servo");

        rightIntakeServo =
                hardwareMap.get(CRServo.class, "right_intake_servo");

        leftDrive.setDirection(DcMotor.Direction.REVERSE);
        rightDrive.setDirection(DcMotor.Direction.FORWARD);

        leftDrive.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        rightDrive.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        intake.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        leftDrive.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        rightDrive.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        leftDrive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        rightDrive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        intake.setPower(0);
        leftIntakeServo.setPower(0);
        rightIntakeServo.setPower(0);
    }

    private boolean timeAvailable() {

        if (isStopRequested()) {
            return false;
        }

        if (matchTimer.seconds() >= AUTO_TIME_LIMIT) {
            telemetry.addLine("Autonomous time limit reached.");
            telemetry.update();
            stopAll();
            return false;
        }

        return true;
    }

    private void drive(double inches, double power) {

        int ticks = (int) (inches * TICKS_PER_INCH);

        moveDrive(ticks, ticks, power);

        telemetry.addData("Drive", "%.1f inches", inches);
        telemetry.update();
    }

    private void turn(double degrees, double power) {

        double arcDistance =
                Math.PI * TRACK_WIDTH * degrees / 360.0;

        int ticks = (int) (arcDistance * TICKS_PER_INCH);

        moveDrive(ticks, -ticks, power);

        telemetry.addData("Turn", "%.1f degrees", degrees);
        telemetry.update();
    }

    private void moveDrive(int leftTicks, int rightTicks, double power) {

        if (!opModeIsActive() || !timeAvailable()) {
            stopDrive();
            return;
        }

        leftDrive.setTargetPosition(
                leftDrive.getCurrentPosition() + leftTicks
        );

        rightDrive.setTargetPosition(
                rightDrive.getCurrentPosition() + rightTicks
        );

        leftDrive.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        rightDrive.setMode(DcMotor.RunMode.RUN_TO_POSITION);

        double safePower = Math.abs(power);

        leftDrive.setPower(safePower);
        rightDrive.setPower(safePower);

        ElapsedTime movementTimer = new ElapsedTime();

        double distanceInches =
                Math.abs(leftTicks) / TICKS_PER_INCH;

        double timeoutSeconds =
                Math.max(2.0, distanceInches * 0.3 + 2.0);

        while (opModeIsActive()
                && timeAvailable()
                && (leftDrive.isBusy() || rightDrive.isBusy())
                && movementTimer.seconds() < timeoutSeconds) {

            telemetry.addData("Left Position",
                    leftDrive.getCurrentPosition());

            telemetry.addData("Right Position",
                    rightDrive.getCurrentPosition());

            telemetry.addData("Movement Time",
                    "%.1f", movementTimer.seconds());

            telemetry.update();

            idle();
        }

        stopDrive();

        leftDrive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        rightDrive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
    }

    private void collect(double seconds) {

        if (!opModeIsActive() || !timeAvailable()) {
            return;
        }

        telemetry.addLine("Collecting...");
        telemetry.update();

        intake.setPower(1.0);
        leftIntakeServo.setPower(1.0);
        rightIntakeServo.setPower(1.0);

        ElapsedTime collectionTimer = new ElapsedTime();

        while (opModeIsActive()
                && timeAvailable()
                && collectionTimer.seconds() < seconds) {

            idle();
        }

        intake.setPower(0);
        leftIntakeServo.setPower(0);
        rightIntakeServo.setPower(0);
    }

    private void shoot() {

        // Placeholder until shooting mechanism is confirmed.
        telemetry.addLine("Shoot action not configured yet.");
        telemetry.update();

        sleep(250);
    }

    private void pollenReleasePlaceholder() {

        // Placeholder until pollen-release mechanism is confirmed.
        telemetry.addLine("Pollen release not configured yet.");
        telemetry.update();

        sleep(250);
    }

    private void stopDrive() {

        if (leftDrive != null) {
            leftDrive.setPower(0);
        }

        if (rightDrive != null) {
            rightDrive.setPower(0);
        }
    }

    private void stopAll() {

        stopDrive();

        if (intake != null) {
            intake.setPower(0);
        }

        if (leftIntakeServo != null) {
            leftIntakeServo.setPower(0);
        }

        if (rightIntakeServo != null) {
            rightIntakeServo.setPower(0);
        }
    }
}
