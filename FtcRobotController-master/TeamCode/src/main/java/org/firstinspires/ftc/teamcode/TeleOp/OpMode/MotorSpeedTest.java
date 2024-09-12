package org.firstinspires.ftc.teamcode.TeleOp.OpMode;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.TeleOp.MecannumDriveHandler;

@TeleOp
public class MotorSpeedTest extends LinearOpMode
{
    @Override
    public void runOpMode() throws InterruptedException
    {
        MecannumDriveHandler drive = new MecannumDriveHandler(hardwareMap);
        telemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());

        waitForStart();

        while (opModeIsActive())
        {
            drive.MoveRobotCentric(gamepad1.left_stick_x, -gamepad1.left_stick_y, gamepad1.right_stick_x);

            double[] velocities = drive.getVelocities();

            telemetry.addData("lFD", velocities[0]);
            telemetry.addData("rFD", velocities[1]);
            telemetry.addData("lBD", velocities[2]);
            telemetry.addData("rBD", velocities[3]);

            telemetry.update();
        }
    }
}
