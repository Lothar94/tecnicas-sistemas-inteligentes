#include "myPlannerLite.h"
#include "geometry_msgs/Twist.h"
#include <tf/transform_datatypes.h> //para transformar quaternions en ángulos, necesario en odomCallBack

LocalPlanner::LocalPlanner() :LocalPlanner(0.01, 3.5, 0.05, 0.02, 1.0, 0.01){
  /*
  //Parámetros de configuración (radio, spread, alpha) del campo actractivo.
  CAMPOATT.radius = 0.01; CAMPOATT.spread = 3.5; CAMPOATT.intens = 0.05;
  //Parámetros de configuración (radio, spread, beta)del campo repulsivo.
  CAMPOREP.radius = 0.02; CAMPOREP.spread = 1.0; CAMPOREP.intens = 0.01;
  //Posición del objetivo
  posGoal.x = posGoal.y = 0;
  //Posición actual
  pos.x = pos.y = 0;
  //Angulo (en radianes) de orientación del robot
  yaw = 0;
  //Componente del campo atractivo
  deltaGoal.x = deltaGoal.y = 0;
  //Componente del campo repulsivo (para todos los obstáculos)
  deltaObst.x = deltaObst.y = 0;
  //Componente total
  delta.x = delta.y = 0;
  //Velocidad angular
  v_angular = v_lineal = 0;

  //Advertise a new publisher for the simulated robot's velocity command topic
  commandPub = node.advertise<geometry_msgs::Twist>("cmd_vel", 10);

  //Subscribe to the simulated robot's laser scan topic
  laserSub = node.subscribe("base_scan", 1, &LocalPlanner::scanCallBack, this);
  odomSub = node.subscribe("base_pose_ground_truth", 1, &LocalPlanner::odomCallBack, this);
  */
}

LocalPlanner::LocalPlanner(double att_r, double att_s, double att_i, double rep_r, double rep_s, double rep_i){
  //Parámetros de configuración (radio, spread, alpha) del campo actractivo.
  CAMPOATT.radius = att_r; CAMPOATT.spread = att_s; CAMPOATT.intens = att_i;
  //Parámetros de configuración (radio, spread, beta)del campo repulsivo.
  CAMPOREP.radius = rep_r; CAMPOREP.spread = rep_s; CAMPOREP.intens = rep_i;
  //Posición del objetivo
  posGoal.x = posGoal.y = 0;
  //Posición actual
  pos.x = pos.y = 0;
  //Angulo (en radianes) de orientación del robot
  yaw = 0;
  //Componente del campo atractivo
  deltaGoal.x = deltaGoal.y = 0;
  //Componente del campo repulsivo (para todos los obstáculos)
  deltaObst.x = deltaObst.y = 0;
  //Componente total
  delta.x = delta.y = 0;
  //Velocidad angular
  v_angular = v_lineal = 0;

  //Advertise a new publisher for the simulated robot's velocity command topic
  commandPub = node.advertise<geometry_msgs::Twist>("cmd_vel", 10);

  //Subscribe to the simulated robot's laser scan topic
  laserSub = node.subscribe("base_scan", 1, &LocalPlanner::scanCallBack, this);
  odomSub = node.subscribe("base_pose_ground_truth", 1, &LocalPlanner::odomCallBack, this);
}

void LocalPlanner::odomCallBack(const nav_msgs::Odometry::ConstPtr& msg){
  //Obtengo la posición
  double xPos=msg->pose.pose.position.x;
  double yPos=msg->pose.pose.position.y;
  //La asigno a LocalPlanner::pos
  pos.x = xPos;
  pos.y = yPos;

  //También la guardo en LocalPlanner::odometria
  odometria.pose.pose.position.x = xPos;
  odometria.pose.pose.position.y = yPos;

  //Get Quaternion anglular information
  double x=msg->pose.pose.orientation.x;
  double y=msg->pose.pose.orientation.y;
  double z=msg->pose.pose.orientation.z;
  double w=msg->pose.pose.orientation.w;

  //La guardo en la odometría
  odometria.pose.pose.orientation.x = x;
  odometria.pose.pose.orientation.y = y;
  odometria.pose.pose.orientation.z = z;
  odometria.pose.pose.orientation.w = w;

  //Convierto el quaternion en radianes y guardo el yaw en LocalPlanner::yaw
  yaw=atan2(2*(y*x+w*z),w*w+x*x-y*y-z*z);
  ROS_INFO("POSE: %f %f %f",xPos,yPos,yaw);
};

//Calcula el componente atractivo del campo de potencial
void LocalPlanner::setDeltaAtractivo(){
  double dist = distancia(posGoal, pos);
  double angulo = atan2(posGoal.y - pos.y, posGoal.x - pos.x);

  // Distinguimos 3 casos
  if (dist < CAMPOATT.radius) {
    deltaGoal.x = deltaGoal.y = 0;
  } else if (dist < CAMPOATT.spread + CAMPOATT.radius) {
    deltaGoal.x = CAMPOATT.intens * (dist - CAMPOATT.radius) * cos(angulo);
    deltaGoal.y = CAMPOATT.intens * (dist - CAMPOATT.radius) * sin(angulo);
  } else {
    deltaGoal.x = CAMPOATT.intens * CAMPOATT.spread * cos(angulo);
    deltaGoal.y = CAMPOATT.intens * CAMPOATT.spread * sin(angulo);
  }
}

// Recibe una posición de un obstáculo y calcula el componente repulsivo para ese obstáculo.
// Devuelve los valores en deltaO.x y deltaO.y
void LocalPlanner::getOneDeltaRepulsivo(Tupla obstaculo, Tupla &deltaO){
  double dist = distancia(obstaculo, pos);
  double angulo = atan2(obstaculo.y - pos.y, obstaculo.x - pos.x);

  // Distinguimos 3 casos
  if (dist < CAMPOREP.radius) {
    deltaO.x = -signo(cos(angulo)) * std::numeric_limits<double>::infinity();
    deltaO.y = -signo(sin(angulo)) * std::numeric_limits<double>::infinity();
  } else if (dist <= CAMPOREP.spread + CAMPOREP.radius) {
    deltaO.x = -CAMPOREP.intens * (CAMPOREP.spread + CAMPOREP.radius - dist) * cos(angulo);
    deltaO.y = -CAMPOREP.intens * (CAMPOREP.spread + CAMPOREP.radius - dist) * sin(angulo);
  } else {
    deltaO = {0, 0};
  }
}

//Calcula la componente total repulsiva como suma de las componentes repulsivas para cada obstáculo.
void LocalPlanner::setTotalRepulsivo(){
    // Ponemos las componentes a 0.
    deltaObst.x = deltaObst.y = 0;
    // Variables auxiliares
    Tupla aux;
    int size = posObs.size();
    // Calcula la componente a cada obstáculo y la suma a deltaObst.
    for (int i = 0; i < size; i++)
      getOneDeltaRepulsivo(posObs.at(i), aux);
      deltaObst.x += aux.x; deltaObst.y += aux.y;
    }
}

void LocalPlanner::scanCallBack(const sensor_msgs::LaserScan::ConstPtr& scan){
  //Find the closest range between the defined minimum and maximum angles
  int minIndex = ceil((MIN_SCAN_ANGLE_RAD - scan->angle_min) / scan->angle_increment);
  int maxIndex = floor((MAX_SCAN_ANGLE_RAD - scan->angle_min) / scan->angle_increment);

  //Limpio el vector de posiciones de obstaculos
  if (posObs.size() > 0)
    posObs.clear();

  //Bearing indica el ángulo de cada muestra
  double bearing = MIN_SCAN_ANGLE_RAD;

  //Calculamos la posición de cada obstáculo dada por la muestra láser.
  float closestRange = scan->ranges[minIndex];

  for (int currIndex = minIndex + 1; currIndex <= maxIndex; currIndex++){
    Tupla obstaculo;
    obstaculo.x =pos.x + scan->ranges[currIndex]*cos(yaw+bearing);
    obstaculo.y =pos.y + scan->ranges[currIndex]*sin(yaw+bearing);
    //ROS_INFO("Obstaculo %d en posición (%f,%f)",currIndex,obstaculo.x, obstaculo.y);
    posObs.push_back(obstaculo);
    bearing += scan->angle_increment;
  }
}

//Normaliza un ángulo al intervalo [-PI, PI]
double normalize(double angle){
  angle = fmod(angle + M_PI, 2*M_PI);
  if (angle < 0)
    angle += 2*M_PI;
  return angle - M_PI;
}

//Calcula la velocidad angular
void LocalPlanner::setv_Angular(){
  double angulo = atan2(delta.y, delta.x);
  double diferencia_normalizada = normalize(angulo-yaw);

  if ((0 <= diferencia_normalizada) and (diferencia_normalizada <= M_PI)){
    if (diferencia_normalizada > V_ANGULAR_CTE)
      v_angular = V_ANGULAR_CTE;
    else
      v_angular = (diferencia_normalizada < EPSILON_ANGULAR)? 0: diferencia_normalizada;
  }
  else{
    if (diferencia_normalizada < (-1)*V_ANGULAR_CTE)
      v_angular = (-1)*V_ANGULAR_CTE;
    else
      v_angular = (diferencia_normalizada > (-1)*EPSILON_ANGULAR)? 0: diferencia_normalizada;
  }
}

//Calcula la velocidad lineal
void LocalPlanner::setv_Lineal(){
  v_lineal =  sqrt(delta.x*delta.x + delta.y*delta.y);
}

//Determina que el objetivo se ha alcanzado cuando ambas velocidades son 0.
bool LocalPlanner::goalAchieved(){
  return (v_angular == 0 and v_lineal == 0);
}
