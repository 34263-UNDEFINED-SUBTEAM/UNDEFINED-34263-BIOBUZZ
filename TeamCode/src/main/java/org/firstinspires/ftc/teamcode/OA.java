
package org.firstinspires.ftc.teamcode;

import com.qualcomm.hardware.gobilda.GoBildaPinpointDriver;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;

@Autonomous(name = "OA - Sequential Autonomous", group = "Autonomous")
public class OA extends LinearOpMode {

    // Drive motors
    private DcMotor frontLeft;
    private DcMotor frontRight;
    private DcMotor backLeft;
    private DcMotor backRight;

    // Shooter
    private DcMotor pollenShooter;

    // Pinpoint
    private GoBildaPinpointDriver pinpoint;

    // Shooter settings
    private static final double SHOOTER_POWER = 1.0;
    private static final long SHOOTER_TIME_MS = 10000;

    // Drive settings
    private static final double DRIVE_POWER_LIMIT = 0.50;
    private static final double TURN_POWER_LIMIT = 0.30;

    private static final double DRIVE_KP = 0.045;
    private static final double TURN_KP = 0.015;

    private static final double DRIVE_TOLERANCE_IN = 0.75;
    private static final double TURN_TOLERANCE_DEG = 1.5;

    private static final double DRIVE_TIMEOUT_SECONDS = 8.0;
    private static final double TURN_TIMEOUT_SECONDS = 5.0;

    // Pinpoint mounting offsets.
    // Replace these with your measured physical offsets if they are not zero.
    private static final double X_OFFSET_MM = 0.0;
    private static final double Y_OFFSET_MM = 0.0;

    @Override
    public void runOpMode() {

        initializeHardware();

        if (!hardwareReady()) {
            telemetry.addLine("Hardware initialization failed.");
            telemetry.update();
            return;
        }

        telemetry.addLine("OA initialized.");
        telemetry.addLine("Shooter: 10 seconds");
        telemetry.addLine("Then: 16 inches forward");
        telemetry.addLine("Then: 45 degrees left");
        telemetry.addLine("Then: 48 inches forward");
        telemetry.update();

        waitForStart();

        if (isStopRequested()) {
            stopAll();
            return;
        }

        try {

            // STEP 1: Run shooter for exactly 10 seconds.
            telemetry.addLine("STEP 1: SHOOTER RUNNING");
            telemetry.update();

            pollenShooter.setPower(SHOOTER_POWER);

            long shooterStart = System.currentTimeMillis();

            while (opModeIsActive()
                    && System.currentTimeMillis() - shooterStart < SHOOTER_TIME_MS) {

                telemetry.addData(
                        "Shooter time remaining",
                        "%.1f seconds",
                        (SHOOTER_TIME_MS -
                                (System.currentTimeMillis() - shooterStart)) / 1000.0
                );

                telemetry.update();
                idle();
            }

            // Stop shooter before beginning any movement.
            pollenShooter.setPower(0);

            if (!opModeIsActive()) {
                return;
            }

            // STEP 2: Drive forward 16 inches.
            telemetry.addLine("STEP 2: DRIVE FORWARD 16 INCHES");
            telemetry.update();

            if (!driveForward(16.0)) {
                telemetry.addLine("Drive step failed or timed out.");
                telemetry.update();
                return;
            }

            if (!opModeIsActive()) {
                return;
            }

            // STEP 3: Turn left 45 degrees.
            telemetry.addLine("STEP 3: TURN LEFT 45 DEGREES");
            telemetry.update();

            if (!turnLeft(45.0)) {
                telemetry.addLine("Turn step failed or timed out.");
                telemetry.update();
                return;
            }

            if (!opModeIsActive()) {
                return;
            }

            // STEP 4: Drive forward 48 inches.
            telemetry.addLine("STEP 4: DRIVE FORWARD 48 INCHES");
            telemetry.update();

            if (!driveForward(48.0)) {
                telemetry.addLine("Final drive step failed or timed out.");
                telemetry.update();
                return;
            }

            telemetry.addLine("OA SEQUENCE COMPLETE");
            telemetry.update();

        } finally {
            stopAll();
        }
    }

    private void initializeHardware() {

        frontLeft = hardwareMap.get(DcMotor.class, "front_left_drive");
        frontRight = hardwareMap.get(DcMotor.class, "front_right_drive");
        backLeft = hardwareMap.get(DcMotor.class, "back_left_drive");
        backRight = hardwareMap.get(DcMotor.class, "back_right_drive");

        pollenShooter = hardwareMap.get(DcMotor.class, "pollen_shooter");

        pinpoint = hardwareMap.get(
                GoBildaPinpointDriver.class,
                "pinpoint"
        );

        // Drive motor directions.
        frontLeft.setDirection(DcMotor.Direction.REVERSE);
        backLeft.setDirection(DcMotor.Direction.REVERSE);

        frontRight.setDirection(DcMotor.Direction.FORWARD);
        backRight.setDirection(DcMotor.Direction.FORWARD);

        // Configure motors.
        setDriveZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        frontLeft.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        frontRight.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        backLeft.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        backRight.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        pollenShooter.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        pollenShooter.setPower(0);

        // Configure Pinpoint.
        pinpoint.setOffsets(
                X_OFFSET_MM,
                Y_OFFSET_MM,
                DistanceUnit.MM
        );

        pinpoint.setEncoderResolution(
                GoBildaPinpointDriver.GoBildaOdometryPods.goBILDA_4_BAR_POD
        );

        pinpoint.setEncoderDirections(
                GoBildaPinpointDriver.EncoderDirection.FORWARD,
                GoBildaPinpointDriver.EncoderDirection.FORWARD
        );

        pinpoint.resetPosAndIMU();

        // Allow the IMU and odometry readings to initialize.
        for (int i = 0; i < 10; i++) {
            pinpoint.update();
            sleep(20);
        }
    }

    private boolean hardwareReady() {

        if (frontLeft == null
                || frontRight == null
                || backLeft == null
                || backRight == null
                || pollenShooter == null
                || pinpoint == null) {
            return false;
        }

        return true;
    }

    private void setDriveZeroPowerBehavior(
            DcMotor.ZeroPowerBehavior behavior) {

        frontLeft.setZeroPowerBehavior(behavior);
        frontRight.setZeroPowerBehavior(behavior);
        backLeft.setZeroPowerBehavior(behavior);
        backRight.setZeroPowerBehavior(behavior);
    }

    // Drive forward relative to the robot's heading when this method starts.
    private boolean driveForward(double inches) {

        pinpoint.update();

        double startX = pinpoint.getPosX(DistanceUnit.INCH);
        double startY = pinpoint.getPosY(DistanceUnit.INCH);
        double startHeading = pinpoint.getHeading(AngleUnit.RADIANS);

        double targetX = startX + inches * Math.cos(startHeading);
        double targetY = startY + inches * Math.sin(startHeading);

        double startTime = getRuntime();

        while (opModeIsActive()) {

            pinpoint.update();

            double currentX = pinpoint.getPosX(DistanceUnit.INCH);
            double currentY = pinpoint.getPosY(DistanceUnit.INCH);
            double currentHeading = pinpoint.getHeading(AngleUnit.RADIANS);

            double errorX = targetX - currentX;
            double errorY = targetY - currentY;

            double forwardError =
                    errorX * Math.cos(startHeading)
                            + errorY * Math.sin(startHeading);

            double headingError = normalizeDegrees(
                    Math.toDegrees(startHeading - currentHeading)
            );

            if (Math.abs(forwardError) <= DRIVE_TOLERANCE_IN
                    && Math.abs(headingError) <= TURN_TOLERANCE_DEG) {

                stopDrive();
                return true;
            }

            if (getRuntime() - startTime > DRIVE_TIMEOUT_SECONDS) {
                stopDrive();
                return false;
            }

            double forwardPower = clip(
                    forwardError * DRIVE_KP,
                    -DRIVE_POWER_LIMIT,
                    DRIVE_POWER_LIMIT
            );

            double turnPower = clip(
                    headingError * TURN_KP,
                    -TURN_POWER_LIMIT,
                    TURN_POWER_LIMIT
            );

            // Positive turn power is a left turn.
            setDrivePower(
                    forwardPower - turnPower,
                    forwardPower + turnPower,
                    forwardPower - turnPower,
                    forwardPower + turnPower
            );

            telemetry.addData("Drive target X", "%.2f", targetX);
            telemetry.addData("Drive target Y", "%.2f", targetY);
            telemetry.addData("Current X", "%.2f", currentX);
            telemetry.addData("Current Y", "%.2f", currentY);
            telemetry.addData("Forward error", "%.2f", forwardError);
            telemetry.addData("Heading error", "%.2f", headingError);
            telemetry.update();

            idle();
        }

        stopDrive();
        return false;
    }

    private boolean turnLeft(double degrees) {

        pinpoint.update();

        double startHeading =
                pinpoint.getHeading(AngleUnit.DEGREES);

        double targetHeading = startHeading + degrees;

        double startTime = getRuntime();

        while (opModeIsActive()) {

            pinpoint.update();

            double currentHeading =
                    pinpoint.getHeading(AngleUnit.DEGREES);

            double error = normalizeDegrees(
                    targetHeading - currentHeading
            );

            if (Math.abs(error) <= TURN_TOLERANCE_DEG) {
                stopDrive();
                return true;
            }

            if (getRuntime() - startTime > TURN_TIMEOUT_SECONDS) {
                stopDrive();
                return false;
            }

            double turnPower = clip(
                    error * TURN_KP,
                    -TURN_POWER_LIMIT,
                    TURN_POWER_LIMIT
            );

            // Positive turn power is a left turn.
            setDrivePower(
                    -turnPower,
                    turnPower,
                    -turnPower,
                    turnPower
            );

            telemetry.addData("Target heading", "%.2f", targetHeading);
            telemetry.addData("Current heading", "%.2f", currentHeading);
            telemetry.addData("Turn error", "%.2f", error);
            telemetry.update();

            idle();
        }

        stopDrive();
        return false;
    }

    private void setDrivePower(
            double fl,
            double fr,
            double bl,
            double br) {

        double max = Math.max(
                1.0,
                Math.max(
                        Math.max(Math.abs(fl), Math.abs(fr)),
                        Math.max(Math.abs(bl), Math.abs(br))
                )
        );

        frontLeft.setPower(fl / max);
        frontRight.setPower(fr / max);
        backLeft.setPower(bl / max);
        backRight.setPower(br / max);
    }

    private double normalizeDegrees(double angle) {

        while (angle > 180.0) {
            angle -= 360.0;
        }

        while (angle < -180.0) {
            angle += 360.0;
        }

        return angle;
    }

    private double clip(double value, double min, double max) {
        return Math.max(min, Math.min(max, value));
    }

    private void stopDrive() {
        frontLeft.setPower(0);
        frontRight.setPower(0);
        backLeft.setPower(0);
        backRight.setPower(0);
    }

    private void stopAll() {
        stopDrive();

        if (pollenShooter != null) {
            pollenShooter.setPower(0);
        }
    }
}
