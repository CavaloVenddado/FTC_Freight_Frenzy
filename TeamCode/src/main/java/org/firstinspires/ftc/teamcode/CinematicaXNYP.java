package org.firstinspires.ftc.teamcode;

public class CinematicaXNYP {
    /**
     * codigo escrito pela equipe cavalo vendado 16786 temporada freght frenzy dia 2/2/2022
     * este codigo realiza a cinemática de um  braço mecanico de 2 eixos utilizando trigonometria
     *
     */

    double a = 0.30; //distancia entre o cotovelo e o pulso
    double b = 0.18; //distancia entre o ombro e o cotovelo
    private double T2;
    private double T1;
    public void setPos(double x2, double y2) { //inicio da funcao cinematica
        T2 = Math.acos(((x2*x2) + (y2*y2) - (a*a) - (b*b))/(2*a*b)); //COTOVELO
        T1 = 180 - (180 - (Math.atan2(y2, x2))) + (Math.acos(((x2*x2) + (y2*y2) + (a*a) - (b*b))/(2*(Math.sqrt((x2*x2) + (y2*y2)))*a))); //OMBRO
    }
    // parte do código responsavel por retornar o robo, ao código principal
    public double getT2() {return T2;}
    public double getT1() {return T1;}

}