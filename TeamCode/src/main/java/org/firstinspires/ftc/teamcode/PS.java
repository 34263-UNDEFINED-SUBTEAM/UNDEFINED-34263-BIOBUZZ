
package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.util.ElapsedTime;

@Autonomous(name = "PS - POLLEN STORM", group = "Competition")
public class PS extends RouteAutoBase {

    @Override
    public void runOpMode() {
        initializeRobot();

        telemetry.addLine("PS - POLLEN STORM READY");
        telemetry.addLine("Route 1");
        telemetry.update();

        waitForStart();
        if (isStopRequested()) return;

        matchTimer.reset();

        // 1. Release nectar from flower.
        pollenReleasePlaceholder();

        // 2. Shoot all currently held balls.
        shoot(1.5);

        // 3. Move toward flowers and collect pollen.
        drive(20, 0.40);
        turn(35, 0.35);
        drive(18, 0.40);
        collect(2.0);

        // 4. Shoot collected balls.
        shoot(1.5);

        // 5. Move toward garden and collect balls.
        turn(-70, 0.35);
        drive(30, 0.45);
        collect(3.0);

        stopAll();
        telemetry.addData("PS Status", "Route complete");
        telemetry.addData("Time", "%.1f sec", matchTimer.seconds());
        telemetry.update();
    }
}

/*
 * Shared autonomous base class.
 * GS.java uses this same class.
 */
abstract class RouteAutoBase extends LinearOpMode {

    protected DcMotor leftDrive;
    protected DcMotor rightDrive;
    protected DcMotor intake;

    protected CRServo leftShooter;
    protected CRServo rightShooter;

    protected ElapsedTime matchTimer = new ElapsedTime();

    // CALIBRATE THESE VALUES BEFORE COMPETITION.
    // 45 is only a starting estimate, NOT a confirmed motor value.
    protected static final double TICKS_PER_INCH = 45.0;
    protected static final double TRACK_WIDTH = 14.0;

    protected static final double MATCH_LIMIT = 30.0;

    protected void initializeRobot() {
        leftDrive = hardwareMap.get(DcMotor.class, "left_drive");
        rightDrive = hardwareMap.get(DcMotor.class, "right_drive");
        intake = hardwareMap.get(DcMotor.class, "intake");

        leftShooter = hardwareMap.get(CRServo.class, "left_intake_servo");
        rightShooter = hardwareMap.get(CRServo.class, "right_intake_servo");

        // Verify these directions on blocks before driving.
        leftDrive.setDirection(DcMotor.Direction.REVERSE);
        rightDrive.setDirection(DcMotor.Direction.FORWARD);

        leftDrive.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        rightDrive.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        intake.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        leftDrive.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        rightDrive.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        leftDrive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        rightDrive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        stopAll();
    }

    protected boolean timeAvailable() {
        return opModeIsActive()
                && matchTimer.seconds() < MATCH_LIMIT;
    }

    protected void drive(double inches, double power) {
        if (!timeAvailable()) return;

        int ticks = (int)(inches * TICKS_PER_INCH);

        leftDrive.setTargetPosition(leftDrive.getCurrentPosition() + ticks);
        rightDrive.setTargetPosition(rightDrive.getCurrentPosition() + ticks);

        leftDrive.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        rightDrive.setMode(DcMotor.RunMode.RUN_TO_POSITION);

        leftDrive.setPower(Math.abs(power));
        rightDrive.setPower(Math.abs(power));

        while (timeAvailable()
                && (leftDrive.isBusy() || rightDrive.isBusy())) {
            telemetry.addData("Action", "Driving");
            telemetry.addData("Time", "%.1f", matchTimer.seconds());
            telemetry.update();
            idle();
        }

        stopDrive();
        setDriveMode(DcMotor.RunMode.RUN_USING_ENCODER);
    }

    protected void turn(double degrees, double power) {
        if (!timeAvailable()) return;

        double arc = Math.PI * TRACK_WIDTH * degrees / 360.0;
        int ticks = (int)(arc * TICKS_PER_INCH);

        leftDrive.setTargetPosition(leftDrive.getCurrentPosition() + ticks);
        rightDrive.setTargetPosition(rightDrive.getCurrentPosition() - ticks);

        leftDrive.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        rightDrive.setMode(DcMotor.RunMode.RUN_TO_POSITION);

        leftDrive.setPower(Math.abs(power));
        rightDrive.setPower(Math.abs(power));

        while (timeAvailable()
                && (leftDrive.isBusy() || rightDrive.isBusy())) {
            telemetry.addData("Action", "Turning");
            telemetry.update();
            idle();
        }

        stopDrive();
        setDriveMode(DcMotor.RunMode.RUN_USING_ENCODER);
    }

    protected void collect(double seconds) {
        if (!timeAvailable()) return;

        intake.setPower(0.85);

        double end = Math.min(
                matchTimer.seconds() + seconds,
                MATCH_LIMIT
        );

        while (timeAvailable() && matchTimer.seconds() < end) {
            telemetry.addData("Action", "Collecting");
            telemetry.update();
            idle();
        }

        intake.setPower(0);
    }

    protected void shoot(double seconds) {
        if (!timeAvailable()) return;

        // Assumes both CR servos are shooter rollers.
        leftShooter.setPower(1.0);
        rightShooter.setPower(-1.0);

        double end = Math.min(
                matchTimer.seconds() + seconds,
                MATCH_LIMIT
        );

        while (timeAvailable() && matchTimer.seconds() < end) {
            telemetry.addData("Action", "Shooting");
            telemetry.update();
            idle();
        }

        leftShooter.setPower(0);
        rightShooter.setPower(0);
    }

    protected void pollenReleasePlaceholder() {
        telemetry.addLine("Pollen release: no configured actuator");
        telemetry.update();
    }

    protected void stopDrive() {
        leftDrive.setPower(0);
        rightDrive.setPower(0);
    }

    protected void setDriveMode(DcMotor.RunMode mode) {
        leftDrive.setMode(mode);
        rightDrive.setMode(mode);
    }

    protected void stopAll() {
        if (leftDrive != null) leftDrive.setPower(0);
        if (rightDrive != null) rightDrive.setPower(0);
        if (intake != null) intake.setPower(0);
        if (leftShooter != null) leftShooter.setPower(0);
        if (rightShooter != null) rightShooter.setPower(0);
    }
}
