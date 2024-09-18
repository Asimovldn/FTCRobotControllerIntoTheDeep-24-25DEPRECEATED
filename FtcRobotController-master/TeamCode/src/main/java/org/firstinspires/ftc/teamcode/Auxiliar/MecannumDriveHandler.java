package org.firstinspires.ftc.teamcode.Auxiliar;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.IMU;
import com.qualcomm.robotcore.hardware.VoltageSensor;

public class MecannumDriveHandler
{

    DcMotorEx lBD;
    DcMotorEx lFD;
    DcMotorEx rBD;
    DcMotorEx rFD;

    IMU imu;

    public MecannumDriveHandler(HardwareMap hardwareMap)
    {
        // Seta os motores à suas respectivas variáveis
        lBD = hardwareMap.get(DcMotorEx.class, "left_back");
        lFD = hardwareMap.get(DcMotorEx.class, "left_front");
        rBD = hardwareMap.get(DcMotorEx.class, "right_back");
        rFD = hardwareMap.get(DcMotorEx.class, "right_front");

        // Reseta os encoderes
        lBD.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        lFD.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        rBD.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        rFD.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        // Seta o modo dos motores
        lBD.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        lFD.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        rBD.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        rFD.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        // Inverte direção dos motores
        lBD.setDirection(DcMotorSimple.Direction.FORWARD);
        lFD.setDirection(DcMotorSimple.Direction.FORWARD);
        rBD.setDirection(DcMotorSimple.Direction.REVERSE);
        rFD.setDirection(DcMotorSimple.Direction.REVERSE);

        // Parar quando não há potência
        lBD.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        lFD.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        rBD.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        rFD.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        // Inicializa imu
        imu = hardwareMap.get(IMU.class, "imu");
        imu.initialize(DriveConstants.imuParameters);

    }

    // Esse método faz o robô se movimentar em uma linha reta em relação à arena
    public void moveOnStraightLine(double xDistance, double yDistance)
    {

    }

    public void Analog(double x, double y, double r)
    {
        double denominator = Math.max(Math.abs (y) + Math.abs(x) + Math.abs(r), 1);

        lBD.setPower ( (y - x + r) / denominator);
        lFD.setPower ( (y + x + r) / denominator);
        rFD.setPower ( (y - x - r) / denominator);
        rBD.setPower ( (y + x - r) / denominator);

    }

    public void FielCentric(double x , double y , double r)
    {
        Vector2D vector = new Vector2D(x,y);
        double angle = 1;

        Vector2D rotVector = Vector2D.rotateVector(vector, angle);

        Analog(rotVector.x, rotVector.y, r );



    }
}
