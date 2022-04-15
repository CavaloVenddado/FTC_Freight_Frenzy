package org.firstinspires.ftc.teamcode;

@Deprecated
public class CinematicaXPYP {
    /**
     * codigo escrito pela equipe cavalo vendado 16786 temporada freght frenzy dia 2/2/2022
     * este codigo realiza a cinemática de um  braço mecanico de 2 eixos utilizando trigonometria
     *
     */

    double a = 0.30; //distancia entre o cotovelo e o pulso
    double b = 0.18; //distancia entre o ombro e o cotovelo
    private double T2;
    private double T1;
    private double T2Numerador;
    private double T2Denominador;
    private double T1Numerador;
    private double T1Denominador;
    public void setPos(double x2, double y2) { //inicio da funcao cinematica
        T2Numerador = Math.acos((x2*x2) + (y2*y2) - (a*a) - (b*b));
        T2Denominador = 2*a*b;
        T2 = T2Numerador / T2Denominador; //COTOVELO

        T1Numerador = Math.atan2(y2, x2) + Math.acos((x2*x2) + (y2*y2) + (a*a) - (b*b));
        T1Denominador = (2*(Math.sqrt((x2*x2) + (y2*y2)))*a);
        T1 = T1Numerador / T1Denominador; //OMBRO
    }
    // parte do código responsavel por retornar o robo, ao código principal
    public double getT2() {return T2;}
    public double getT1() {return T1;}

}
