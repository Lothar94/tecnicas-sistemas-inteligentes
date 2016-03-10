#ifndef LOCAL PLANNER_H
#define LOCAL PLANNER_H

#include "ros/ros.h"
#include "sensor_msgs/LaserScan.h"
#include "nav_msgs/Odometry.h"
#include "geometry_msgs/Twist.h"
#include <move_base_msgs/MoveBaseAction.h>

struct Tupla {double x; double y;};
struct PFConf {double radius; double spread; double intens;};

template <typename T> int signo(T val) {
    return  (val/abs(val));
    }

class LocalPlanner
{
    public:
        Tupla posGoal;  //Posición del objetivo
        Tupla pos;      //Posición actual
        double yaw;     //Angulo (en radianes) de orientación del robot
        Tupla deltaGoal;//Componente del campo atractivo
        Tupla deltaObst;//Componente del campo repulsivo (para todos los obstáculos)
        Tupla delta;    //Componente total
        double v_angular; //velocidad angular
        double v_lineal;  //velocidad lineal
        std::vector<Tupla> posObs;    //Vector que contiene las posiciones de los obstáculos.
                                        //Calculado en scanCallback.
        nav_msgs::Odometry odometria;      //guarda el último mensaje de odometría recibido

        PFConf CAMPOATT;  //Parámetros de configuración (radio, spread, alpha) del campo atractivo.
        PFConf CAMPOREP;  //Parámetros de configuración (radio, spread, beta)del campo repulsivo.
        const static double TOLERANCIA = 0.009;   //Valor a partir del cual consideramos que el robot está e
                                                  //en la posición objetivo (ver setDeltaAtractivo)
        const static double V_ANGULAR_CTE = M_PI/8;  //Valor de la velocidad angular constante.
        const static double EPSILON_ANGULAR = 0.0009; //Valor a partir del cual entendemos que el robot está en la orientación deseada
        const static double MIN_SCAN_ANGLE_RAD = -30.0/180*M_PI;
        const static double MAX_SCAN_ANGLE_RAD = +30.0/180*M_PI;
        LocalPlanner(); //constructor.
        void setGoal(const move_base_msgs::MoveBaseGoalConstPtr& goal) {
            posGoal.x = goal->target_pose.pose.position.x;
            posGoal.y = goal->target_pose.pose.position.y;
            }

        bool goalAchieved();    //Devuelve true cuando se ha alcanzado el objetivo
        void setDeltaAtractivo();
        void getOneDeltaRepulsivo(Tupla posO, Tupla &deltaO);
        void setTotalRepulsivo();
        void setDeltaTotal(){
            delta.x = deltaGoal.x + deltaObst.x;
            delta.y = deltaGoal.y + deltaObst.y;
            };
        void setv_Angular();
        void setv_Lineal();
        double distancia(Tupla src, Tupla dst) {
            return sqrt((src.x - dst.x) * (src.x - dst.x) +
                        (src.y - dst.y) * (src.y - dst.y));
            }
        void setSpeed() {
        //rellenar y enviar Twist.
            geometry_msgs::Twist mensajeTwist;
            mensajeTwist.linear.x = v_lineal;
            mensajeTwist.angular.z =v_angular;
            commandPub.publish(mensajeTwist);
            }

    protected:
    private:
        ros::NodeHandle node;
        ros::Publisher commandPub;  //Publicador de velocidades
        ros::Subscriber laserSub;   //Suscriptor del scan laser
        ros::Subscriber odomSub;    //Suscriptor de la odometría.

        void scanCallBack(const sensor_msgs::LaserScan::ConstPtr& scan);
        void odomCallBack(const nav_msgs::Odometry::ConstPtr& msg);

template <typename T> int signo(T val){
  return  (val/abs(val));
}

class LocalPlanner{
public:
  //Posición del objetivo
  Tupla posGoal;
  //Posición actual
  Tupla pos;
  //Angulo (en radianes) de orientación del robot
  double yaw;
  //Componente del campo atractivo
  Tupla deltaGoal;
  //Componente del campo repulsivo (para todos los obstáculos)
  Tupla deltaObst;
  //Componente total
  Tupla delta;
  //velocidad angular
  double v_angular;
  //velocidad lineal
  double v_lineal;
  //Vector que contiene las posiciones de los obstáculos.
  //Calculado en scanCallback.
  std::vector<Tupla> posObs;
  //guarda el último mensaje de odometría recibido
  nav_msgs::Odometry odometria;

  // = {0.01,3,5,0.07};//Parámetros de configuración (radio, spread, alpha) del campo actractivo.
  PFConf CAMPOATT;
  //(0,01,1,0,01);//Parámetros de configuración (radio, spread, beta)del campo repulsivo.
  PFConf CAMPOREP;
  //Valor a partir del cual consideramos que el robot está e
  //en la posición objetivo (ver setDeltaAtractivo)
  const static double TOLERANCIA = 0.009;
  //Valor de la velocidad angular constante.
  const static double V_ANGULAR_CTE = M_PI/8;
  //Valor a partir del cual entendemos que el robot está en la orientación deseada
  const static double EPSILON_ANGULAR = 0.0009;
  const static double MIN_SCAN_ANGLE_RAD = -30.0/180*M_PI;
  const static double MAX_SCAN_ANGLE_RAD = +30.0/180*M_PI;

  LocalPlanner();

  void setGoal(const move_base_msgs::MoveBaseGoalConstPtr& goal){
    posGoal.x = goal->target_pose.pose.position.x;
    posGoal.y = goal->target_pose.pose.position.y;
  }

  //Devuelve true cuando se ha alcanzado el objetivo
  bool goalAchieved();

  void setDeltaAtractivo();

  void getOneDeltaRepulsivo(Tupla posO, Tupla &deltaO);

  void setTotalRepulsivo();

  void setDeltaTotal(){
    delta.x = deltaGoal.x + deltaObst.x;
    delta.y = deltaGoal.y + deltaObst.y;
  }

  void setv_Angular();

  void setv_Lineal();

  double distancia(Tupla src, Tupla dst){
    return sqrt((src.x - dst.x) * (src.x - dst.x) + (src.y - dst.y) * (src.y - dst.y));
  }

  void setSpeed(){
    //rellenar y enviar Twist.
    geometry_msgs::Twist mensajeTwist;
    mensajeTwist.linear.x = v_lineal;
    mensajeTwist.angular.z =v_angular;
    commandPub.publish(mensajeTwist);
  }
private:
  ros::NodeHandle node;
  ros::Publisher commandPub;  //Publicador de velocidades
  ros::Subscriber laserSub;   //Suscriptor del scan laser
  ros::Subscriber odomSub;    //Suscriptor de la odometría.

  void scanCallBack(const sensor_msgs::LaserScan::ConstPtr& scan);

  void odomCallBack(const nav_msgs::Odometry::ConstPtr& msg);
};

#endif // LOCAL PLANNER_H
