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
import com.acmerobotics.roadrunner.trajectory.constraints.TrajectoryVelocityConstraint;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DistanceSensor;
import com.qualcomm.robotcore.hardware.NormalizedColorSensor;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.robotcore.external.navigation.CurrentUnit;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.teamcode.Cinematica_5;
import org.firstinspires.ftc.teamcode.autonomo.visionpipelines.DetectorHSVEDGE;
import org.firstinspires.ftc.teamcode.drive.SampleMecanumDrive;
import org.firstinspires.ftc.teamcode.trajectorysequence.TrajectorySequence;
import org.openftc.easyopencv.OpenCvCamera;
import org.openftc.easyopencv.OpenCvCameraFactory;
import org.openftc.easyopencv.OpenCvCameraRotation;

@Autonomous(name="AutoFC_B_Blue_CUBE", group="Auto")
public class AutoFC_B_Blue_CUBE extends LinearOpMode {
    OpenCvCamera camera;
    private DcMotorEx Carrossel = null;
    private DcMotorEx motorOmbro = null;
    private DcMotorEx motorCotovelo = null;
    private Servo servoPulso = null;
    private Servo servoGarra = null;
    private Servo servobandeira = null;
    NormalizedColorSensor colorSensor;
    DistanceSensor distSensor;
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
        motorCotovelo.setVelocity(300); //(MAX VEL 600)
        motorCotovelo.setCurrentAlert(4.4, CurrentUnit.AMPS);//current limit
        motorCotovelo.setVelocityPIDFCoefficients(40.00, 0, 0, 60);
        motorOmbro.setVelocityPIDFCoefficients(1, 0, 1, 25.6);
        motorOmbro.setDirection(DcMotorEx.Direction.REVERSE); //ANTI QUEBRA OMBRO

        servoPulso = hardwareMap.get(Servo.class,"ServoPunho");
        servoGarra = hardwareMap.get(Servo.class,"ServoGarra");
        servobandeira = hardwareMap.get(Servo.class,"ServoBandeira");

        servobandeira.setPosition(0);
        servoPulso.setPosition(1);
        closeClaw();

        //distance sensor
        colorSensor = hardwareMap.get(NormalizedColorSensor.class, "sensor_color");
        distSensor = ((DistanceSensor) colorSensor);

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

        Pose2d startPose = new Pose2d(15,62, Math.toRadians(270));
        final Pose2d hubPose = new Pose2d(-12,47, Math.toRadians(270));

        TrajectorySequence toShippingHub = drive.trajectorySequenceBuilder(startPose)
                .lineToLinearHeading(hubPose)
                .build();

        TrajectorySequence toArmazem = drive.trajectorySequenceBuilder(toShippingHub.end())
                .lineToLinearHeading(new Pose2d(0, 64, Math.toRadians(0)))
                //.strafeTo(new Vector2d(0,-62))
                //.strafeTo(new Vector2d(30,65))
                .lineToLinearHeading(new Pose2d(30, 65, Math.toRadians(0)))
                .build();

        TrajectorySequence reverseArmazem = drive.trajectorySequenceBuilder(toArmazem.end())
                .setReversed(true)
                .lineToLinearHeading(new Pose2d(0,64, Math.toRadians(0)))
                //.strafeTo(new Vector2d(0,-50))
                .setReversed(false)
                .lineToLinearHeading(hubPose)
                .build();

        TrajectorySequence deliverLower = drive.trajectorySequenceBuilder(startPose)
                .lineToLinearHeading(new Pose2d(-12,55, hubPose.getHeading()))
                .addTemporalMarker(() -> {setArm(-0.13, -0.13, Math.toRadians(268));})//posição baixa
                .waitSeconds(1)
                .lineToLinearHeading(hubPose)
                .build();

        TrajectorySequence vazar = drive.trajectorySequenceBuilder(toShippingHub.end())
                .lineToLinearHeading(new Pose2d(0, 64, Math.toRadians(0)))
                .lineToLinearHeading(new Pose2d(40, 65, Math.toRadians(0)))
                .lineToLinearHeading(new Pose2d(40,40, Math.toRadians(0)))
                .build();

        while (!isStarted()){
            telemetry.addData("Detection results", colorfilter.getAnalysis());telemetry.addData("Posição Motor Ombro: ",motorOmbro.getCurrentPosition());
            telemetry.addData("dist", distSensor.getDistance(DistanceUnit.CM));
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
        //start
        drive.setPoseEstimate(startPose);
        setArm(-0.137, 0.029, Math.toRadians(200));
        if(analysis == 1){
            drive.followTrajectorySequence(deliverLower);// special sequence for lower traj
            openClaw();
            sleep(2000);
        }else if(analysis == 2){
            drive.followTrajectorySequence(toShippingHub);
            setArm(-0.15, 0.022, Math.toRadians(260.5));//meio
            sleep(2000);
            openClaw();
            sleep(2000);
        }else if(analysis == 3){
            drive.followTrajectorySequence(toShippingHub);
            setArm(-0.2, 0.28, Math.toRadians(250));//cima
            sleep(1500);
            openClaw();
            sleep(300);
            setArm(0.1, 0.28, Math.toRadians(256));//cima pra tras
            sleep(500);
        }
        setArm(-0.137, 0.029, Math.toRadians(200));
        //drive.followTrajectorySequence(toCarossel);
        drive.followTrajectorySequence(toArmazem);

        /*
        agora ja entregamos na altura certa e estamos no armazém,
        vamos pegar um cubo:
         */
        getACube(drive, toArmazem.end());
        /*
        pegamos e voltamos, agora dar de ré até a torre.
         */
        drive.followTrajectorySequence(reverseArmazem);
        if(Math.abs(drive.getLastError().getX()) < 15 && Math.abs(drive.getLastError().getY()) < 15) {
            //colocar no mais alto
            setArm(-0.2, 0.28, Math.toRadians(250));//cima
            sleep(1700);
            servobandeira.setPosition(0);
            openClaw();
            sleep(300);
            setArm(0.1, 0.28, Math.toRadians(256));//cima
            sleep(500);
            //voltar pro armazém
            setArm(-0.137, 0.029, Math.toRadians(200));
            sleep(1000);
        }else{
            openClaw();
            servoPulso.setPosition(1);
        }
        drive.followTrajectorySequence(vazar);
        sleep(1000);
    }

    /**
     * goes forward until detected a cube, then closes claw and goes back to param pose.
     */
    private void getACube(SampleMecanumDrive drive, Pose2d startingPose){
        motorCotovelo.setTargetPosition(0);
        motorOmbro.setTargetPosition(0);
        servoPulso.setPosition(0);
        servoGarra.setPosition(0.6);
        //create constants
        TrajectoryVelocityConstraint slowSpd = (v, pose2d, pose2d1, pose2d2) -> 10;

        TrajectorySequence getCubeTraj = drive.trajectorySequenceBuilder(startingPose)
                .setVelConstraint(slowSpd)
                .forward(30)
                .build();


        //grabs a cube
        sleep(500);
        drive.followTrajectorySequenceAsync(getCubeTraj);
        while(drive.isBusy()){
            drive.update();
            double dist = distSensor.getDistance(DistanceUnit.CM);
            telemetry.addData("buscando cubo", drive.isBusy());
            telemetry.addData("distancia", dist);
            telemetry.update();
            if(isStopRequested()){
                drive.breakFollowing();
                return;
            }
            if((!Double.isNaN(dist)) && dist < 8){
                servobandeira.setPosition(1);
                break;
            }
        }
        drive.breakFollowing();
        drive.update();
        closeClaw();
        telemetry.addData("cubo encontrado!", "wow");
        telemetry.update();
        sleep(500);
        setArm(-0.087, -0.021, Math.toRadians(222.3));
        sleep(300);
        setArm(-0.137, 0.029, Math.toRadians(200));
        TrajectorySequence goBack = drive.trajectorySequenceBuilder(drive.getPoseEstimate())
                .resetVelConstraint()
                .lineToLinearHeading(startingPose)
                .build();

        telemetry.addData("Voltando...", "será?");
        telemetry.update();
        drive.followTrajectorySequence(goBack);
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
        if (!Double.isNaN(cotovelo) && cotovelo > Math.toRadians(0) && cotovelo < Math.toRadians(180)) {
            motorCotovelo.setTargetPosition((int) (cotovelo * fatorCotovelo));
        }

        if (!Double.isNaN(ombro) && ombro > Math.toRadians(0) && ombro < Math.toRadians(180)) {
            motorOmbro.setTargetPosition((int) (ombro * fatorOmbro));
        }
        if (!Double.isNaN(pulso) && pulso*fatorPulso >= 0 && pulso*fatorPulso <= 1){
            servoPulso.setPosition(pulso*fatorPulso);
        }
    }
}
