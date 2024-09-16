@TeleOp(name="teste TeleOp", group="Linear OpMode")

public class testeTeleOp extends LinearOpMode {

    MecannumDriveHandler drive;

    @Override
    public void runOpMode()
    {
        drive = new MecannumDriveHandler(hardwareMap);

        waitForStart();

        while (opModeIsActive())
        {
            drive.Analog(gamepad1.left_stick_x, -gamepad1.left_stick_y, gamepad1.right_stick_x);
        }
    }
}