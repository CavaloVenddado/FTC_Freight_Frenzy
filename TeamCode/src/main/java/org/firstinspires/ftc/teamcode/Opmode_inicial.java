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

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
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
    private double phi;
    private DigitalChannel botao1ombro;
    private DigitalChannel botao2cotovelo;
    private int Velocidade_Carrossel;
    private ElapsedTime tempodegiro = new ElapsedTime();

    /*
    nestas proximas duas linhas foram definidas os fatores do cotovelo e do ombro
    responsaveis por transformar os valores do motores do ombro e cotovelo em radianos,
    respectivamente
     */
    private double fatorCotovelo = (-264/ Math.toRadians(180)); //-264 = 180°
    private double fatorOmbro = (5600 / Math.toRadians(180)); //5600 = 180°
    private double fatorPulso = (1 / Math.toRadians(180));
    @Override
    public void runOpMode() {

        telemetry.addData("Status", "Initialized");
        telemetry.update();
        telemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());


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

        motorFrontLeft.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        motorBackLeft.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        motorFrontRight.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        motorBackRight.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

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
        motorOmbro.setVelocity(600); //600               08-04   800
        motorCotovelo.setVelocity(800); //500           08-04   1000
        motorCotovelo.setVelocityPIDFCoefficients(40.00, 0, 0, 13.6);
        motorOmbro.setVelocityPIDFCoefficients(14.26, 0.1, 0, 22.6);
        /*
        Motor Cotovelo   PIDF = 40.00 , 0 , 0 , 13.6
        Motor Ombro      PIDF = 14.26 , 0.1 , 5 , 22.6
         */
        servoPulso = hardwareMap.get(Servo.class,"ServoPunho");
        servoGarra = hardwareMap.get(Servo.class,"ServoGarra");
        servoPulso.setPosition(1);
        // Most robots need the motor on one side to be reversed to drive forward
        // Reverse the motor that runs backwards when connected directly to the battery
        Carrossel.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        motorBackRight.setDirection(DcMotorEx.Direction.REVERSE);
        motorFrontRight.setDirection(DcMotorEx.Direction.REVERSE);
        motorOmbro.setDirection(DcMotorEx.Direction.REVERSE);
        // Wait for the game to start (driver presses PLAY)

        Cinematica_5 braco2 = new Cinematica_5();

        //a partir daqui o codigo é iniciado no celular
        waitForStart();
        runtime.reset();
        // run until the end of the match (driver presses STOP)

        /*
        Seguencia de enquanto responsavel por operar o atuador do carrosel
        Se o botão b estiver pressionado, o motor irá girar!
         */
        PosY = -0.01;
        PosX = 0.1;
        phi = Math.toRadians(270);
        while (opModeIsActive()) {
            if (gamepad1.b == true) {
                Velocidade_Carrossel = 1500;
                Carrossel.setVelocity(Velocidade_Carrossel);
                tempodegiro.reset();
                if(tempodegiro.seconds() > 3){
                    Velocidade_Carrossel = 3000;
                    Carrossel.setVelocity(Velocidade_Carrossel);
                }
            }
            else {
                Carrossel.setVelocity(0);
                Velocidade_Carrossel = 0;
            }
            /*
            Declaração de variaveis referentes à movimentação das rodas.
             */
            double velocity = (gamepad1.right_trigger * 0.80) + 0.20;
            double y = gamepad1.left_stick_y * velocity;
            double x = gamepad1.left_stick_x * -1.1 * velocity;
            double rx = -gamepad1.right_stick_x * velocity;

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

            PosY = PosY + gamepad2.right_stick_y * -0.01;
            PosX = PosX + gamepad2.right_stick_x * -0.01;
            phi = phi + (gamepad2.left_stick_x * 0.1);
            if (phi < Math.toRadians(90)){
                phi = Math.toRadians(90);
            }
            if (phi > Math.toRadians(360)){
                phi = Math.toRadians(360);
            }

            /*
            Essa sequencia de if é encarregada de definir o alcance maximo do braço
             */
            if (PosY > 2){
                PosY = 2;
            }
            if (PosY < -1){
                PosY = -1;
            }

            if (PosX > 2) {
                PosX = 2;
            }
            if (PosX < -2) {
                PosX = -2;
            }
            if (phi > 280) {
                phi = 280;
            }
            if (gamepad2.dpad_down){
                motorOmbro.setVelocity(400);
                motorCotovelo.setVelocity(600);
            }
            if (gamepad2.dpad_up) {
                motorOmbro.setVelocity(600);
                motorCotovelo.setVelocity(800);
            }

            if (gamepad2.a == true){
                phi = Math.toRadians(260);
                PosX = -0.1;
                PosY = 0.01;
            }
            if (gamepad2.x == true){
                phi = Math.toRadians(254);
                PosX = -0.1;
                PosY = 0.089;
            }
            if (gamepad2.y == true){
                phi = Math.toRadians(256);//246
                PosX = 0.0212;//-0.123
                PosY = 0.1798;//0.227
            }
            if (gamepad2.b == true){
                phi = Math.toRadians(222);//230
                PosX = -0.04;//-0.17
                PosY = 0.31;//0.338
            }
            if (gamepad2.dpad_right == true) {
                phi = Math.toRadians(295.9083);
                PosX = -0.06;
                PosY = -0.02;
                //motorOmbro.setVelocity(-800); //wtf
            }
            if (gamepad2.dpad_left == true) {
                PosY = -0.01;
                PosX = 0.1;
                phi = Math.toRadians(270);
            }

            braco2.setPos(PosX, PosY, phi);



            //aplicar posições no robo
            double ombro = braco2.getTe1();
            double cotovelo = braco2.getTe2();
            double pulso = Math.toRadians(180) - braco2.getTe3();
            if (cotovelo != Double.NaN && cotovelo > Math.toRadians(0) && cotovelo < Math.toRadians(180)) {
                motorCotovelo.setTargetPosition((int) (cotovelo * fatorCotovelo));
            }

            if (ombro != Double.NaN && ombro > Math.toRadians(0) && ombro < Math.toRadians(180)) {
                motorOmbro.setTargetPosition((int) (ombro * fatorOmbro));
            }
            if (pulso != Double.NaN && pulso*fatorPulso >= 0 && pulso*fatorPulso <= 1){
                servoPulso.setPosition(pulso*fatorPulso);
            }
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
            telemetry.addData("Te1: ", Math.toDegrees(braco2.getTe1()));
            telemetry.addData("Te2: ", Math.toDegrees(braco2.getTe2()));
            telemetry.addData("Te3: ", Math.toDegrees(braco2.getTe3()));
            telemetry.addData("CurrentOmbro: ", motorOmbro.getCurrent(CurrentUnit.AMPS));
            telemetry.addData("CurrentCotovelo: ", motorCotovelo.getCurrent(CurrentUnit.AMPS));
            telemetry.addData("PHI: ", Math.toDegrees(phi));
            telemetry.update();
        }
    }
}

