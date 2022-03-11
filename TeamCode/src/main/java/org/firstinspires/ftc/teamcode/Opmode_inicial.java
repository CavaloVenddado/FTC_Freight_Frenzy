/* Copyright (c) 2017 FIRST. All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without modification,
 * are permitted (subject to the limitations in the disclaimer below) provided that
 * the following conditions are met:
 *
 * Redistributions of source code must retain the above copyright notice, this list
 * of conditions and the following disclaimer.
 *
 * Redistributions in binary form must reproduce the above copyright notice, this
 * list of conditions and the following disclaimer in the documentation and/or
 * other materials provided with the distribution.
 *
 * Neither the name of FIRST nor the names of its contributors may be used to endorse or
 * promote products derived from this software without specific prior written permission.
 *
 * NO EXPRESS OR IMPLIED LICENSES TO ANY PARTY'S PATENT RIGHTS ARE GRANTED BY THIS
 * LICENSE. THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 * "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO,
 * THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
 * ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE
 * FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
 * DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
 * SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
 * CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY,
 * OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
 * OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 */

package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.DigitalChannel;
import com.qualcomm.robotcore.hardware.PIDFCoefficients;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.navigation.CurrentUnit;


/**
 * This file contains an minimal example of a Linear "OpMode". An OpMode is a 'program' that runs in either
 * the autonomous or the teleop period of an FTC match. The names of OpModes appear on the menu
 * of the FTC Driver Station. When an selection is made from the menu, the corresponding OpMode
 * class is instantiated on the Robot Controller and executed.
 *
 * This particular OpMode just executes a basic Tank Drive Teleop for a two wheeled robot
 * It includes all the skeletal structure that all linear OpModes contain.
 *
 * Use Android Studios to Copy this Class, and Paste it into your team's code folder with a new name.
 * Remove or comment out the @Disabled line to add this opmode to the Driver Station OpMode list
 */

@TeleOp(name="Opmode_Inicial", group="Linear Opmode")
public class Opmode_inicial extends LinearOpMode {

    // Declare OpMode members.
    /*
    Nesta parte do codigo são declaradas as variaveis dos motores, botões e servos instalados no robo
     */
    private ElapsedTime runtime = new ElapsedTime();
    private DcMotorEx Carrossel = null;
    private DcMotorEx motorFrontLeft = null;
    private DcMotorEx motorBackLeft = null;
    private DcMotorEx motorFrontRight = null;
    private DcMotorEx motorBackRight = null;
    private DcMotorEx motorOmbro = null;
    private DcMotorEx motorCotovelo = null;
    private Servo servoPulso = null;
    private Servo servoGarra = null;
    private double PosY;
    private double PosX;
    private DigitalChannel botao1ombro;
    private DigitalChannel botao2cotovelo;

    /*
    nestas proximas duas linhas foram definidas os fatores do cotovelo e do ombro
    responsaveis por transformar os valores do motores do ombro e cotovelo em radianos,
    respectivamente
     */
    private double fatorCotovelo = (-264/ Math.toRadians(180)); //-264 = 180°
    private double fatorOmbro = (5600 / Math.toRadians(180)); //5600 = 180°
    @Override
    public void runOpMode() {
        telemetry.addData("Status", "Initialized");
        telemetry.update();


        // Initialize the hardware variables. Note that the strings used here as parameters
        // to 'get' must correspond to the names assigned during the robot configuration
        // step (using the FTC Robot Controller app on the phone).
        /*
        em seguida é criado o hardwaremap/mapa de hardware cujo o objetivo é nomear os motores,
        botões e servos.
        é também definido os modos em que servos e motores vão operar
        */
        Carrossel = hardwareMap.get(DcMotorEx.class, "Carrossel");
        motorFrontLeft = hardwareMap.get(DcMotorEx.class,"motorFrontLeft");
        motorBackLeft = hardwareMap.get(DcMotorEx.class,"motorBackLeft");
        motorFrontRight = hardwareMap.get(DcMotorEx.class,"motorFrontRight");
        motorBackRight = hardwareMap.get(DcMotorEx.class,"motorBackRight");

        botao1ombro = hardwareMap.get(DigitalChannel.class, "Botao1Ombro");
        botao2cotovelo = hardwareMap.get(DigitalChannel.class, "Botao2Cotovelo");
        botao1ombro.setMode(DigitalChannel.Mode.INPUT);
        botao2cotovelo.setMode(DigitalChannel.Mode.INPUT);

        motorCotovelo = hardwareMap.get(DcMotorEx.class,"Cotovelo");
        motorOmbro = hardwareMap.get(DcMotorEx.class,"Ombro");
        motorOmbro.setTargetPosition(0);
        motorCotovelo.setTargetPosition(0);
        motorCotovelo.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        motorOmbro.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        motorOmbro.setVelocity(600); //600
        motorCotovelo.setVelocity(500); //500
        motorCotovelo.setVelocityPIDFCoefficients(26.00, 0, 2, 13.6);
        motorOmbro.setVelocityPIDFCoefficients(15.26, 0, 5, 22.6);
        servoPulso = hardwareMap.get(Servo.class,"ServoPunho");
        servoGarra = hardwareMap.get(Servo.class,"ServoGarra");

        // Most robots need the motor on one side to be reversed to drive forward
        // Reverse the motor that runs backwards when connected directly to the battery
        Carrossel.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        motorBackRight.setDirection(DcMotorEx.Direction.REVERSE);
        motorFrontRight.setDirection(DcMotorEx.Direction.REVERSE);
        motorOmbro.setDirection(DcMotorEx.Direction.REVERSE);
        // Wait for the game to start (driver presses PLAY)
        Robotic_Arm braco = new Robotic_Arm();
        Arm_Robotic graveto = new Arm_Robotic();
        CinematicaXPYP bracopos = new CinematicaXPYP();
        CinematicaXNYP braconeg = new CinematicaXNYP();

        //a partir daqui o codigo é iniciado no celular
        waitForStart();
        runtime.reset();
        // run until the end of the match (driver presses STOP)

        /*
        Seguencia de enquanto responsavel por operar o atuador do carrosel
        Se o botão b estiver pressionado, o motor irá girar!
         */
        PosY = 0.5;
        PosX = 0.1;
        while (opModeIsActive()) {
            if (gamepad1.b == true) {
                Carrossel.setVelocity(-2500);
            } else {
                Carrossel.setVelocity(0);
            }
            /*
            Declaração de variaveis referentes à movimentoção das rodas.
             */
            double velocity = gamepad1.right_trigger;
            double y = gamepad1.left_stick_y * velocity;
            double x = gamepad1.left_stick_x * -1.1 * velocity;
            double rx = -gamepad1.right_stick_x * velocity;

            /*
            Declaração de variaveis referentes a posição do braço
             */
            double y2 = gamepad2.right_stick_y;
            double x2 = gamepad2.right_stick_x;

            /*
            Codigo responsavel pela movimentação do robô
             */
            double denominator = Math.max(Math.abs(y) + Math.abs(x) + Math.abs(rx), 1);
            double frontLeftPower = (y + x + rx) / denominator;
            double backLeftPower = (y - x + rx) / denominator;
            double frontRightPower = (y - x - rx) / denominator;
            double backRightPower = (y + x - rx) / denominator;

            motorFrontLeft.setPower(frontLeftPower);
            motorBackLeft.setPower(backLeftPower);
            motorFrontRight.setPower(frontRightPower);
            motorBackRight.setPower(backRightPower);

            PosY = PosY + gamepad2.right_stick_y * -0.001;
            PosX = PosX + gamepad2.left_stick_x * 0.001;

            /*
            Essa sequencia de if é encarregada de definir o alcance maximo do braço
             */
            if (PosY > 3){
                PosY = 3;
            }
            if (PosY < 0.1){
                PosY = 0.1;
            }

            if (PosX > 3) {
                PosX = 3;
            }
            if (PosX < -3) {
                PosX = -3;
            }

            braco.setPos(PosX, PosY);
            graveto.setPos(PosX, PosY);
            bracopos.setPos(PosX, PosY);
            braconeg.setPos(PosX, PosY);
            // Motores do braço se dirigem para o angolo exato levando em consideração os ajustes
            double ombro = Math.toRadians(180) - graveto.getT1();
            double cotovelo = Math.toRadians(180) + graveto.getT2();
            /* Cinematica Julio
            if (PosX > 0) {

                double ombro = bracopos.getT1();
                double cotovelo = bracopos.getT2();
                if (cotovelo != Double.NaN) {
                    motorCotovelo.setTargetPosition((int) (cotovelo * fatorCotovelo));
                }

                if (ombro != Double.NaN) {
                    motorOmbro.setTargetPosition((int) (ombro * fatorOmbro));
                }
            }
            if (PosX <= 0) {
                double ombro = bracopos.getT1();
                double cotovelo = bracopos.getT2();
                if (cotovelo != Double.NaN) {
                    motorCotovelo.setTargetPosition((int) (cotovelo * fatorCotovelo));
                }

                if (ombro != Double.NaN) {
                    motorOmbro.setTargetPosition((int) (ombro * fatorOmbro));
                }
            }
            */

            // Choose to drive using either Tank Mode, or POV Mode
            // Comment out the method that's not used.  The default below is POV.

            if (gamepad2.right_bumper == true){
                servoGarra.setPosition(0);
            }
            if (gamepad2.left_bumper == true){
                servoGarra.setPosition(1);
            }

            // Sequencia responsavel por exibir no monitor os valores importantes do codigo.
            telemetry.addData("Status", "Run Time: " + runtime.toString());
            telemetry.addData("Posição X2: ", PosX);
            telemetry.addData("Posição Y2: ", PosY);
            telemetry.addData("Botão Ombro: ", botao1ombro.getState());
            telemetry.addData("Botão Cotovelo: ", botao2cotovelo.getState());
            telemetry.addData("Posição Motor Ombro: ",motorOmbro.getCurrentPosition());
            telemetry.addData("Posição Motor Cotovelo: ",motorCotovelo.getCurrentPosition());
            telemetry.addData("Target Position Ombro: ", motorOmbro.getTargetPosition());
            telemetry.addData("Target Position Cotovelo: ", motorCotovelo.getTargetPosition());
            telemetry.addData("T1: ", Math.toDegrees(graveto.getT1()));
            telemetry.addData("T2: ", Math.toDegrees(graveto.getT2()));
            telemetry.addData("CurrentOmbro: ", motorOmbro.getCurrent(CurrentUnit.AMPS));
            telemetry.addData("CurrentCotovelo: ", motorCotovelo.getCurrent(CurrentUnit.AMPS));
            telemetry.update();
        }
    }
}

