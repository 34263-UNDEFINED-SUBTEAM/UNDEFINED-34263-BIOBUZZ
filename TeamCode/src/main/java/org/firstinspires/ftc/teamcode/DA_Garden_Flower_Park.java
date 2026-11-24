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

            //Put pollen in the garden

            telemetry.addData(">", "Turn");
            telemetry.update();
            robot.turnTo(-180, 0.45, 0.5);

            telemetry.addData(">", "forward");
            telemetry.update();
            robot.drive( 48, 0.60, 0.25);

            //Intake from flower

            telemetry.addData(">", "forward");
            telemetry.update();
            robot.drive( 84, 0.60, 0.25);

            telemetry.addData(">", "Finished");
            telemetry.update();
        }
    }
}