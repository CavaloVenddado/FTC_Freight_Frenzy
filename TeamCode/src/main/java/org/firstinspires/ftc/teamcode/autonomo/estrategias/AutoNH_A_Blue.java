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

package org.firstinspires.ftc.teamcode.autonomo.estrategias;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.robotcore.external.navigation.CurrentUnit;
import org.firstinspires.ftc.teamcode.Cinematica_5;
import org.firstinspires.ftc.teamcode.autonomo.visionpipelines.DetectorHSVEDGE;
import org.firstinspires.ftc.teamcode.drive.SampleMecanumDrive;
import org.firstinspires.ftc.teamcode.trajectorysequence.TrajectorySequence;
import org.openftc.easyopencv.OpenCvCamera;
import org.openftc.easyopencv.OpenCvCameraFactory;
import org.openftc.easyopencv.OpenCvCameraRotation;

@Autonomous(name="Hamburauto-A-Blue", group="Auto")
public class AutoNH_A_Blue extends LinearOpMode {
    OpenCvCamera camera;
    private DcMotorEx Carrossel = null;
    private DcMotorEx motorOmbro = null;
    private DcMotorEx motorCotovelo = null;
    private Servo servoPulso = null;
    private Servo servoGarra = null;
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
        //init camera
        Carrossel = hardwareMap.get(DcMotorEx.class, "Carrossel");

        motorCotovelo = hardwareMap.get(DcMotorEx.class,"Cotovelo");
        motorOmbro = hardwareMap.get(DcMotorEx.class,"Ombro");
        motorOmbro.setTargetPosition(0);
        motorCotovelo.setTargetPosition(0);
        motorCotovelo.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        motorOmbro.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        motorOmbro.setVelocity(2000); //(MAX VEL 2800)
        motorCotovelo.setVelocity(500); //(MAX VEL 600)
        motorCotovelo.setCurrentAlert(4.4, CurrentUnit.AMPS);//current limit
        motorCotovelo.setVelocityPIDFCoefficients(40.00, 0, 0, 13.6);
        motorOmbro.setVelocityPIDFCoefficients(1, 0, 1, 25.6);
        motorOmbro.setDirection(DcMotorEx.Direction.REVERSE); //ANTI QUEBRA OMBRO

        servoPulso = hardwareMap.get(Servo.class,"ServoPunho");
        servoGarra = hardwareMap.get(Servo.class,"ServoGarra");

        servoPulso.setPosition(1);
        servoGarra.setPosition(0);

        telemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());
        DetectorHSVEDGE colorfilter = new DetectorHSVEDGE();
        int cameraMonitorViewId = hardwareMap.appContext.getResources().getIdentifier("cameraMonitorViewId", "id", hardwareMap.appContext.getPackageName());
        WebcamName webcamName = hardwareMap.get(WebcamName.class, "Webcam 1");
        camera = OpenCvCameraFactory.getInstance().createWebcam(webcamName, cameraMonitorViewId);
        camera.openCameraDeviceAsync(new OpenCvCamera.AsyncCameraOpenListener()
        {
            @Override
            public void onOpened()
            {
                camera.setPipeline(colorfilter);
                camera.startStreaming(1280, 720, OpenCvCameraRotation.UPRIGHT);
            }

            @Override
            public void onError(int errorCode)
            {
                /*
                 * This will be called if the camera could not be opened
                 */
            }
        });
        //init RR
        SampleMecanumDrive drive = new SampleMecanumDrive(hardwareMap);

        Pose2d startPose = new Pose2d(-31,62, Math.toRadians(270));

        //start trajectory
        TrajectorySequence toShippingHub = drive.trajectorySequenceBuilder(startPose)
                .splineTo(new Vector2d(-12,45), Math.toRadians(270))
                .build();
        TrajectorySequence toCarossel = drive.trajectorySequenceBuilder(new Pose2d(-12,45,Math.toRadians(270)))
                .strafeTo(new Vector2d(-65,55))
                .addTemporalMarker(() -> {Carrossel.setPower(1);})
                .waitSeconds(3)
                .addTemporalMarker(() -> {Carrossel.setPower(0);})
                .build();
        TrajectorySequence toArmazem = drive.trajectorySequenceBuilder(new Pose2d(-65,55,Math.toRadians(270)))
                .strafeTo(new Vector2d(-60,55))
                .splineToSplineHeading(new Pose2d(0,65, Math.toRadians(0)), Math.toRadians(0))
                .strafeTo(new Vector2d(40,65))
                .build();
        TrajectorySequence deliverLower = drive.trajectorySequenceBuilder(startPose)
                .splineTo(new Vector2d(-12,55), Math.toRadians(270))
                .addTemporalMarker(() -> {setArm(-0.2, -0.025, Math.toRadians(249));})//posição baixa
                .waitSeconds(1)
                .strafeTo(new Vector2d(-12,45))
                .build();

        //setArm(0.1, -0.01, Math.toRadians(270));
        while (!isStarted()){
            telemetry.addData("Detection results", colorfilter.getAnalysis());telemetry.addData("Posição Motor Ombro: ",motorOmbro.getCurrentPosition());
            telemetry.addData("Posição Motor Ombro: ",motorOmbro.getCurrentPosition());
            telemetry.addData("Posição Motor Cotovelo: ",motorCotovelo.getCurrentPosition());
            telemetry.addData("Target Position Ombro: ", motorOmbro.getTargetPosition());
            telemetry.addData("Target Position Cotovelo: ", motorCotovelo.getTargetPosition());
            telemetry.addData("CurrentOmbro: ", motorOmbro.getCurrent(CurrentUnit.AMPS));
            telemetry.addData("CurrentCotovelo: ", motorCotovelo.getCurrent(CurrentUnit.AMPS));
            telemetry.update();

        }
        int analysis = 0;
        while(analysis == 0 && isStarted()){
            analysis = colorfilter.getAnalysis();
        }
        drive.setPoseEstimate(startPose);
        // coloca o braço na posição desejada
        if(analysis == 1){
            drive.followTrajectorySequence(deliverLower);// special sequence for lower traj
        }else if(analysis == 2){
            drive.followTrajectorySequence(toShippingHub);
            setArm(-0.1, 0.022, Math.toRadians(260.5));//meio
        }else if(analysis == 3){
            drive.followTrajectorySequence(toShippingHub);
            setArm(0.0212, 0.1798, Math.toRadians(256));//cima
        }
        openClaw();
        sleep(2000);
        setArm(-0.137, 0.029, Math.toRadians(200));
        drive.followTrajectorySequence(toCarossel);
        drive.followTrajectorySequence(toArmazem);
        //start

    }
    void openClaw(){
        servoGarra.setPosition(1);
    }
    void closeClaw(){
        servoGarra.setPosition(0);
    }
    void setArm(Double PosX, Double PosY, Double phi){
        Cinematica_5 braco = new Cinematica_5();
        braco.setPos(PosX, PosY, phi);

        //aplicar posições no robo
        double ombro = braco.getTe1();
        double cotovelo = braco.getTe2();
        double pulso = Math.toRadians(180) - braco.getTe3();
        if(pulso < Math.toRadians(0)){
            pulso = Math.toRadians(0);
        }
        if(pulso > Math.toRadians(180)){
            pulso = Math.toRadians(180);
        }
        if (cotovelo != Double.NaN && cotovelo > Math.toRadians(0) && cotovelo < Math.toRadians(180)) {
            motorCotovelo.setTargetPosition((int) (cotovelo * fatorCotovelo));
        }

        if (ombro != Double.NaN && ombro > Math.toRadians(0) && ombro < Math.toRadians(180)) {
            motorOmbro.setTargetPosition((int) (ombro * fatorOmbro));
        }
        if (pulso != Double.NaN && pulso*fatorPulso >= 0 && pulso*fatorPulso <= 1){
            servoPulso.setPosition(pulso*fatorPulso);
        }
    }
}
