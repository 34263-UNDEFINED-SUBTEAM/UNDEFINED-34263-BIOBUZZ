package org.firstinspires.ftc.teamcode;

import com.qualcomm.hardware.limelightvision.LLResult;
import com.qualcomm.hardware.limelightvision.LLResultTypes;
import com.qualcomm.hardware.limelightvision.Limelight3A;
import com.qualcomm.hardware.sparkfun.SparkFunOTOS;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.util.Range;

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

    private boolean usingOTOS = false;
    private boolean usingLimelight = false;

    private static final double SQUARE_SIZE = 60.0;
    private static final double BALL_DISTANCE = 24.0;

    private static final double MAX_POWER = 0.65;
    private static final double MIN_POWER = 0.12;

    private static final double POSITION_KP = 0.035;
    private static final double HEADING_KP = 0.018;

    private static final double POSITION_TOLERANCE = 1.0;
    private static final double HEADING_TOLERANCE = 2.0;
    private static final double TX_TOLERANCE = 1.5;

    private static final double FALLBACK_TICKS_PER_REV = 537.7;
    private static final double WHEEL_DIAMETER = 4.0;

    private static final double TICKS_PER_INCH =
            FALLBACK_TICKS_PER_REV / (Math.PI * WHEEL_DIAMETER);

    @Override
    public void runOpMode() throws InterruptedException {

        setupMotors();
        setupOTOS();
        setupLimelight();

        if (!motorsReady()) {
            telemetry.addLine("Drive motors not found.");
            telemetry.update();
            return;
        }

        telemetry.addData("OTOS", usingOTOS ? "Ready" : "Not found");
        telemetry.addData("Limelight", usingLimelight ? "Ready" : "Not found");
        telemetry.addLine("Ready");
        telemetry.update();

        waitForStart();

        if (isStopRequested()) {
            stopRobot();
            return;
        }

        if (usingOTOS) {
            runSquareWithOTOS();
        } else {
            runSquareWithEncoders();
        }

        stopRobot();

        if (usingLimelight) {
            findAndDriveToBall();
        }

        stopRobot();
    }

    private void setupMotors() {

        frontLeft = findMotor(
                "front_left_drive",
                "frontLeft",
                "front_left",
                "left_front"
        );

        frontRight = findMotor(
                "front_right_drive",
                "frontRight",
                "front_right",
                "right_front"
        );

        backLeft = findMotor(
                "back_left_drive",
                "backLeft",
                "back_left",
                "left_back"
        );

        backRight = findMotor(
                "back_right_drive",
                "backRight",
                "back_right",
                "right_back"
        );

        if (motorsReady()) {
            frontLeft.setDirection(DcMotorSimple.Direction.FORWARD);
            backLeft.setDirection(DcMotorSimple.Direction.FORWARD);

            frontRight.setDirection(DcMotorSimple.Direction.REVERSE);
            backRight.setDirection(DcMotorSimple.Direction.REVERSE);

            setRunMode(DcMotor.RunMode.RUN_USING_ENCODER);
            stopRobot();
        }
    }

    private DcMotor findMotor(String... names) {

        for (String name : names) {
            try {
                return hardwareMap.get(DcMotor.class, name);
            } catch (Exception ignored) {
            }
        }

        return null;
    }

    private boolean motorsReady() {

        return frontLeft != null &&
                frontRight != null &&
                backLeft != null &&
                backRight != null;
    }

    private void setupOTOS() {

        String[] names = {
                "otos",
                "sensor_otos",
                "SparkFunOTOS",
                "sparkfun_otos"
        };

        for (String name : names) {
            try {
                otos = hardwareMap.get(SparkFunOTOS.class, name);

                if (otos.begin()) {

                    otos.setLinearUnit(DistanceUnit.INCH);
                    otos.setAngularUnit(AngleUnit.DEGREES);

                    telemetry.addLine("Calibrating OTOS...");
                    telemetry.update();

                    otos.calibrateImu();

                    otos.resetTracking();
                    otos.setPosition(
                            new SparkFunOTOS.Pose2D(0, 0, 0)
                    );

                    usingOTOS = true;
                    return;
                }

            } catch (Exception ignored) {
            }
        }
    }

    private void setupLimelight() {

        String[] names = {
                "limelight",
                "Limelight",
                "limelight3A"
        };

        for (String name : names) {
            try {

                limelight =
                        hardwareMap.get(Limelight3A.class, name);

                limelight.pipelineSwitch(0);
                limelight.start();

                usingLimelight = true;
                return;

            } catch (Exception ignored) {
            }
        }
    }

    private void runSquareWithOTOS() {

        SparkFunOTOS.Pose2D start = otos.getPosition();

        double x = start.x;
        double y = start.y;
        double heading = start.h;

        moveToOTOS(
                x - SQUARE_SIZE,
                y,
                heading,
                "Left"
        );

        moveToOTOS(
                x - SQUARE_SIZE,
                y + SQUARE_SIZE,
                heading,
                "Forward"
        );

        moveToOTOS(
                x,
                y + SQUARE_SIZE,
                heading,
                "Right"
        );

        moveToOTOS(
                x,
                y,
                heading,
                "Back"
        );
    }

    private boolean moveToOTOS(
            double targetX,
            double targetY,
            double targetHeading,
            String movement) {

        long startTime = System.currentTimeMillis();
        long timeout = 6000;

        while (opModeIsActive()) {

            if (System.currentTimeMillis() - startTime > timeout) {
                stopRobot();
                return false;
            }

            SparkFunOTOS.Pose2D pose = otos.getPosition();

            double errorX = targetX - pose.x;
            double errorY = targetY - pose.y;

            double distance =
                    Math.hypot(errorX, errorY);

            double headingError =
                    AngleUnit.normalizeDegrees(
                            targetHeading - pose.h
                    );

            if (distance <= POSITION_TOLERANCE &&
                    Math.abs(headingError) <= HEADING_TOLERANCE) {

                stopRobot();
                sleep(150);
                return true;
            }

            double headingRadians =
                    Math.toRadians(pose.h);

            double robotX =
                    errorX * Math.cos(headingRadians) +
                            errorY * Math.sin(headingRadians);

            double robotY =
                    -errorX * Math.sin(headingRadians) +
                            errorY * Math.cos(headingRadians);

            double xPower =
                    Range.clip(
                            robotX * POSITION_KP,
                            -MAX_POWER,
                            MAX_POWER
                    );

            double yPower =
                    Range.clip(
                            robotY * POSITION_KP,
                            -MAX_POWER,
                            MAX_POWER
                    );

            double turnPower =
                    Range.clip(
                            headingError * HEADING_KP,
                            -0.25,
                            0.25
                    );

            if (Math.abs(xPower) > 0 &&
                    Math.abs(xPower) < MIN_POWER) {

                xPower =
                        Math.copySign(MIN_POWER, xPower);
            }

            if (Math.abs(yPower) > 0 &&
                    Math.abs(yPower) < MIN_POWER) {

                yPower =
                        Math.copySign(MIN_POWER, yPower);
            }

            driveMecanum(
                    yPower,
                    xPower,
                    turnPower
            );

            telemetry.addData(
                    "Movement",
                    movement
            );

            telemetry.addData(
                    "X",
                    "%.1f / %.1f",
                    pose.x,
                    targetX
            );

            telemetry.addData(
                    "Y",
                    "%.1f / %.1f",
                    pose.y,
                    targetY
            );

            telemetry.addData(
                    "Heading",
                    "%.1f / %.1f",
                    pose.h,
                    targetHeading
            );

            telemetry.update();
        }

        stopRobot();
        return false;
    }

    private void runSquareWithEncoders() {

        driveEncoder(0.55, -SQUARE_SIZE);
        driveEncoder(0.55, SQUARE_SIZE);

        strafeEncoder(0.55, SQUARE_SIZE);
        strafeEncoder(0.55, -SQUARE_SIZE);
    }

    private void driveEncoder(
            double power,
            double inches) {

        int ticks =
                (int) Math.round(
                        inches * TICKS_PER_INCH
                );

        setRunMode(
                DcMotor.RunMode.STOP_AND_RESET_ENCODER
        );

        frontLeft.setTargetPosition(ticks);
        frontRight.setTargetPosition(ticks);
        backLeft.setTargetPosition(ticks);
        backRight.setTargetPosition(ticks);

        setRunMode(
                DcMotor.RunMode.RUN_TO_POSITION
        );

        setPower(Math.abs(power));

        while (opModeIsActive() &&
                (frontLeft.isBusy() ||
                        frontRight.isBusy() ||
                        backLeft.isBusy() ||
                        backRight.isBusy())) {

            telemetry.addData(
                    "Encoder",
                    "Driving"
            );

            telemetry.update();
        }

        stopRobot();
        sleep(150);
    }

    private void strafeEncoder(
            double power,
            double inches) {

        int ticks =
                (int) Math.round(
                        inches * TICKS_PER_INCH
                );

        setRunMode(
                DcMotor.RunMode.STOP_AND_RESET_ENCODER
        );

        frontLeft.setTargetPosition(-ticks);
        backLeft.setTargetPosition(ticks);

        frontRight.setTargetPosition(ticks);
        backRight.setTargetPosition(-ticks);

        setRunMode(
                DcMotor.RunMode.RUN_TO_POSITION
        );

        setPower(Math.abs(power));

        while (opModeIsActive() &&
                (frontLeft.isBusy() ||
                        frontRight.isBusy() ||
                        backLeft.isBusy() ||
                        backRight.isBusy())) {

            telemetry.addData(
                    "Encoder",
                    "Strafing"
            );

            telemetry.update();
        }

        stopRobot();
        sleep(150);
    }

    private void findAndDriveToBall() {

        long startTime =
                System.currentTimeMillis();

        boolean found = false;

        while (opModeIsActive() &&
                System.currentTimeMillis() - startTime < 8000) {

            LLResult result =
                    limelight.getLatestResult();

            if (result != null &&
                    result.isValid()) {

                List<LLResultTypes.DetectorResult>
                        detections =
                        result.getDetectorResults();

                for (LLResultTypes.DetectorResult detection :
                        detections) {

                    String label =
                            detection.getClassName();

                    if (isPinkBall(label)) {
                        found = true;
                        break;
                    }
                }

                if (found) {
                    break;
                }
            }

            driveMecanum(
                    0,
                    0,
                    0.20
            );
        }

        stopRobot();

        if (!found) {

            telemetry.addLine(
                    "Pink ball not found."
            );

            telemetry.update();
            return;
        }

        centerOnBall();

        driveForwardDistance(
                BALL_DISTANCE
        );
    }

    private void centerOnBall() {

        long startTime =
                System.currentTimeMillis();

        while (opModeIsActive() &&
                System.currentTimeMillis() - startTime < 5000) {

            LLResult result =
                    limelight.getLatestResult();

            if (result == null ||
                    !result.isValid()) {

                stopRobot();
                continue;
            }

            double tx = result.getTx();

            if (Math.abs(tx) <= TX_TOLERANCE) {

                stopRobot();
                sleep(200);
                return;
            }

            double turn =
                    Range.clip(
                            tx * 0.02,
                            -0.25,
                            0.25
                    );

            driveMecanum(
                    0,
                    0,
                    turn
            );

            telemetry.addData(
                    "Ball tx",
                    "%.2f",
                    tx
            );

            telemetry.update();
        }

        stopRobot();
    }

    private boolean isPinkBall(String label) {

        if (label == null) {
            return false;
        }

        String name =
                label.toLowerCase();

        return name.contains("pink") &&
                (name.contains("ball") ||
                        name.contains("wiffle"));
    }

    private void driveForwardDistance(
            double inches) {

        if (usingOTOS) {

            SparkFunOTOS.Pose2D pose =
                    otos.getPosition();

            double heading =
                    Math.toRadians(pose.h);

            double targetX =
                    pose.x -
                            Math.sin(heading) *
                                    inches;

            double targetY =
                    pose.y +
                            Math.cos(heading) *
                                    inches;

            moveToOTOS(
                    targetX,
                    targetY,
                    pose.h,
                    "Ball"
            );

            return;
        }

        driveEncoder(
                0.45,
                inches
        );
    }

    private void driveMecanum(
            double forward,
            double strafe,
            double turn) {

        double frontLeftPower =
                forward + strafe + turn;

        double backLeftPower =
                forward - strafe + turn;

        double frontRightPower =
                forward - strafe - turn;

        double backRightPower =
                forward + strafe - turn;

        double max =
                Math.max(
                        1.0,
                        Math.max(
                                Math.abs(frontLeftPower),
                                Math.max(
                                        Math.abs(backLeftPower),
                                        Math.max(
                                                Math.abs(frontRightPower),
                                                Math.abs(backRightPower)
                                        )
                                )
                        )
                );

        frontLeft.setPower(
                frontLeftPower / max
        );

        backLeft.setPower(
                backLeftPower / max
        );

        frontRight.setPower(
                frontRightPower / max
        );

        backRight.setPower(
                backRightPower / max
        );
    }

    private void setPower(double power) {

        frontLeft.setPower(power);
        frontRight.setPower(power);
        backLeft.setPower(power);
        backRight.setPower(power);
    }

    private void setRunMode(
            DcMotor.RunMode mode) {

        frontLeft.setMode(mode);
        frontRight.setMode(mode);
        backLeft.setMode(mode);
        backRight.setMode(mode);
    }

    private void stopRobot() {

        if (frontLeft != null) {
            frontLeft.setPower(0);
        }

        if (frontRight != null) {
            frontRight.setPower(0);
        }

        if (backLeft != null) {
            backLeft.setPower(0);
        }

        if (backRight != null) {
            backRight.setPower(0);
        }
    }
}