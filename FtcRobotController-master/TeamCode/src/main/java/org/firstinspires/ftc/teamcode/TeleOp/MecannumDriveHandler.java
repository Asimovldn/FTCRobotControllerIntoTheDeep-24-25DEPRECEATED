package org.firstinspires.ftc.teamcode.TeleOp;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.IMU;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.YawPitchRollAngles;
import org.firstinspires.ftc.teamcode.Auxiliar.DriveConstantsTeleOp;

public class MecannumDriveHandler
{
    DcMotorEx lBD;
    DcMotorEx lFD;
    DcMotorEx rBD;
    DcMotorEx rFD;

    IMU imu;

    // Inicializador, prepara as variáveis para utilizar depois
    public MecannumDriveHandler(HardwareMap hardwareMap)
    {
        // INICIALIZAR MOTORES
        // Setar motores

        lBD = hardwareMap.get(DcMotorEx.class,"left_back_drive");
        lFD = hardwareMap.get(DcMotorEx.class,"left_front_drive");
        rBD = hardwareMap.get(DcMotorEx.class,"right_back_drive");
        rFD = hardwareMap.get(DcMotorEx.class,"right_front_drive");

        // Setar direção de motores (devido à forma de montagem, motores da esquerda devem mover em direção oposta ao da direita para se moverem para frente ou para trás

        lBD.setDirection(DcMotorSimple.Direction.FORWARD);
        lFD.setDirection(DcMotorSimple.Direction.FORWARD);
        rBD.setDirection(DcMotorSimple.Direction.REVERSE);
        rFD.setDirection(DcMotorSimple.Direction.REVERSE);

        // Reseta o encoder dos motores
        lBD.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        lFD.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        rBD.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        rFD.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        // Define o modo para não rodar com encoder, não será usado no TeleOP
        lBD.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        lFD.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        rBD.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        rFD.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        // Faz com que o motor mantenha a posição quando parado
        lBD.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        lFD.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        rBD.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        rFD.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        // INICIALIZAR IMU
        imu = hardwareMap.get(IMU.class, "imu");
        imu.initialize(DriveConstantsTeleOp.imuParameters); // Pega os parâmetros de um arquivo separado chamado DriveConstants
        imu.resetYaw();
    }

    public void MoveRobotCentric(double x, double y, double turn)
    {
        double max;


        // Calcula potências baseado nos valores para se mover em x, y e girar
        double lFD_Power = y + x + turn;
        double rFD_Power = y - x - turn;
        double lBD_Power = y - x + turn;
        double rBD_Power = y + x - turn;

        // Escolhe o maior valor entre as potências
        max = Math.max(Math.abs(lFD_Power), Math.abs(rFD_Power));
        max = Math.max(max, Math.abs(lBD_Power));
        max = Math.max(max, Math.abs(rBD_Power));

        // Garante que todos os valores estejam entre -1 e 1
        if (max > 1.0)
        {
            lFD_Power /= max;
            rFD_Power /= max;
            lBD_Power /= max;
            rBD_Power /= max;
        }

        // Envia potências para os motores
        lFD.setPower(lFD_Power);
        rFD.setPower(rFD_Power);
        lBD.setPower(lBD_Power);
        rBD.setPower(rBD_Power);
    }

    public void MoveFieldCentric(double x, double y, double turn)
    {
        double max;

        // Define os angulos do imu
        YawPitchRollAngles robotOrientation;
        robotOrientation = imu.getRobotYawPitchRollAngles();

        // Salva o ângulo de guinada
        double angle = robotOrientation.getYaw(AngleUnit.RADIANS);

        // Rotaciona o vetor
        double Fx = Math.cos(angle) * x - Math.sin(angle) * y;
        double Fy = Math.sin(angle) * x + Math.cos(angle) * y;

        // Calcula as potências usando os valores rotacionados
        double lFD_Power = Fy + Fx + turn;
        double rFD_Power = Fy - Fx - turn;
        double lBD_Power = Fy - Fx + turn;
        double rBD_Power = Fy + Fx - turn;

        // Escolhe o maior valor entre as potências
        max = Math.max(Math.abs(lFD_Power), Math.abs(rFD_Power));
        max = Math.max(max, Math.abs(lBD_Power));
        max = Math.max(max, Math.abs(rBD_Power));

        // Garante que todos os valores estejam entre -1 e 1
        if (max > 1.0)
        {
            lFD_Power /= max;
            rFD_Power /= max;
            lBD_Power /= max;
            rBD_Power /= max;
        }

        // Envia potências para os motores
        lFD.setPower(lFD_Power);
        rFD.setPower(rFD_Power);
        lBD.setPower(lBD_Power);
        rBD.setPower(rBD_Power);
    }

}