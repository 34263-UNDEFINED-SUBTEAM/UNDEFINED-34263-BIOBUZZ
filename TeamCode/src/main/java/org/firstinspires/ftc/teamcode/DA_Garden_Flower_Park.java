package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
@Autonomous
public class DA_Garden_Flower_Park extends LinearOpMode
{
    // get an instance of the "Robot" class.
    private SimplifiedOdometryRobot_Pinpoint robot = new SimplifiedOdometryRobot_Pinpoint(this);

    @Override public void runOpMode()
    {

        robot.initialize(true);

        telemetry.addData(">", "Touch Play to run Auto");
        telemetry.update();

        waitForStart();
        robot.resetHeading();

        if (opModeIsActive())
        {
            telemetry.addData(">", "forward");
            telemetry.update();
            robot.drive( 12, 0.60, 0.25);

            telemetry.addData(">", "Turn");
            telemetry.update();
            robot.turnTo(90, 0.45, 0.5);

            robot.setIntakePower(-1);
            telemetry.addData("Intake", -1);

            sleep(1000);

            telemetry.addData(">", "Pause");
            telemetry.update();

            telemetry.addData(">", "Turn");
            telemetry.update();
            robot.turnTo(-180, 0.45, 0.5);

            telemetry.addData(">", "forward");
            telemetry.update();
            robot.drive( 48, 0.60, 0.25);

            robot.setServoPower(1.0);
            robot.setIntakePower(1);
            sleep(1000);
            robot.setServoPower(0.0);
            robot.setIntakePower(0);

            telemetry.addData(">", "forward");
            telemetry.update();
            robot.drive( 84, 0.60, 0.25);

            telemetry.addData(">", "Finished");
            telemetry.update();
        }
    }
}