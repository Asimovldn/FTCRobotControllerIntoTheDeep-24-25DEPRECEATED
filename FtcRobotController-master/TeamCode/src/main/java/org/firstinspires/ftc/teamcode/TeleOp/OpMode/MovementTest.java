package org.firstinspires.ftc.teamcode.TeleOp.OpMode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.TeleOp.MecannumDriveHandler;

@TeleOp
public class MovementTest extends LinearOpMode
{
    MecannumDriveHandler drive;
    @Override
    public void runOpMode() throws InterruptedException
    {
        drive = new MecannumDriveHandler(hardwareMap);

        waitForStart();

        while (opModeIsActive())
        {
            drive.MoveFieldCentric(gamepad1.left_stick_x, gamepad1.left_stick_y, gamepad1.right_stick_x);
        }
    }
}
