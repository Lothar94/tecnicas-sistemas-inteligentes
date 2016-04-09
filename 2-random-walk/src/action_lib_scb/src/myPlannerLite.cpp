#include "myPlannerLite.h"
#include "geometry_msgs/Twist.h"
#include <tf/transform_datatypes.h> //para transformar quaternions en ángulos, necesario en odomCallBack

//Parametros iniciales: LocalPlanner(0.01, 3.5, 0.05, 0.02, 1.0, 0.01)

LocalPlanner::LocalPlanner() :LocalPlanner(0.01, 3.5, 0.05, 0.02, 1.0, 0.01){}

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

  state = running;

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

  // Distinguimos 4 casos
  // Si estamos en el radio del objetivo, hemos llegado a él.
  if (dist < CAMPOATT.radius) {
    deltaGoal.x = deltaGoal.y = 0;
  // Si estamos acercándonos al objetivo, fijamos una velocidad constante (dependiente de los parámetros)
  // para evitar ralentizar el acercamiento al objetivo (que la velocidad no depende de la distancia)
  } else if (dist <= CAMPOATT.spread/2 + CAMPOATT.radius ) {
    deltaGoal.x = CAMPOATT.intens * (CAMPOATT.spread/2 + CAMPOATT.radius) * cos(angulo);
    deltaGoal.y = CAMPOATT.intens * (CAMPOATT.spread/2 + CAMPOATT.radius) * sin(angulo);
  // Si estamos en el campo de atracción, pero alejados, dependemos de la distancia
  } else if (dist <= CAMPOATT.spread + CAMPOATT.radius ) {
      deltaGoal.x = CAMPOATT.intens * (dist - CAMPOATT.radius) * cos(angulo);
      deltaGoal.y = CAMPOATT.intens * (dist - CAMPOATT.radius) * sin(angulo);
  // Fuera del campo de atracción la velocidad es fija.
  } else if (dist > CAMPOATT.spread + CAMPOATT.radius) {
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
  } else if (dist <= CAMPOREP.spread + CAMPOREP.radius && dist >= CAMPOREP.radius) {
    deltaO.x = -CAMPOREP.intens * (CAMPOREP.spread + CAMPOREP.radius - dist) * cos(angulo);
    deltaO.y = -CAMPOREP.intens * (CAMPOREP.spread + CAMPOREP.radius - dist) * sin(angulo);
  } else if (dist > CAMPOREP.spread + CAMPOREP.radius) {
    deltaO.x = 0;
    deltaO.y = 0;
  }
}

//Calcula la componente total repulsiva como suma de las componentes repulsivas para cada obstáculo.
void LocalPlanner::setTotalRepulsivo(){
    // Ponemos las componentes a 0.
    deltaObst.x = deltaObst.y = 0;
    // Variables auxiliares
    Tupla aux;

    // Calcula la componente a cada obstáculo y la suma a deltaObst.
    for (int i = 0; i < posObs.size(); i++) {
      getOneDeltaRepulsivo(posObs.at(i), aux);
      deltaObst.x += aux.x;
      deltaObst.y += aux.y;
    }
}

void LocalPlanner::scanCallBack(const sensor_msgs::LaserScan::ConstPtr& scan){
  //Find the closest range between the defined minimum and maximum angles
  int minIndex = ceil((MIN_SCAN_ANGLE_RAD - scan->angle_min) / scan->angle_increment);
  int maxIndex = floor((MAX_SCAN_ANGLE_RAD - scan->angle_min) / scan->angle_increment);

  std::vector<Tupla> nuevos_obstaculos;

  //Bearing indica el ángulo de cada muestra
  double bearing = MIN_SCAN_ANGLE_RAD;

  //Calculamos la posición de cada obstáculo dada por la muestra láser.
  float closestRange = scan->ranges[minIndex];

  for (int currIndex = minIndex + 1; currIndex <= maxIndex; currIndex++){
    Tupla obstaculo;
    obstaculo.x =pos.x + scan->ranges[currIndex]*cos(yaw+bearing);
    obstaculo.y =pos.y + scan->ranges[currIndex]*sin(yaw+bearing);
    //ROS_INFO("Obstaculo %d en posición (%f,%f)",currIndex,obstaculo.x, obstaculo.y);
    //ROS_INFO_STREAM("X: " << obstaculo.x << "Y: "<< obstaculo.y );
    nuevos_obstaculos.push_back(obstaculo);
    bearing += scan->angle_increment;
  }

  posObs.swap(nuevos_obstaculos);
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

  double diferencia_normalizada = normalize(angulo - yaw);
  int signo = 2 * (diferencia_normalizada >= 0) - 1;
  double abs_diferencia = signo * diferencia_normalizada;

  // resetear el estado
  //state = running;

  if (v_lineal < MIN_LIN_SPEED_FOR_TURNS) {
    v_angular = 0;
  } else if (abs_diferencia > V_ANGULAR_CTE) {
    v_angular = signo * V_ANGULAR_CTE;

    // Notificar que necesitamos girar
    state = turning;
  } else {
    v_angular = (abs_diferencia > EPSILON_ANGULAR) * diferencia_normalizada;
  }
}

// Calcula la velocidad lineal.
// Usar después de calcular la velocidad angular.
void LocalPlanner::setv_Lineal(){
  int sentido = 2 * (state != backwards) - 1;
  v_lineal = sentido * sqrt(delta.x * delta.x + delta.y * delta.y);
}

// Establece la velocidad según el estado en que nos encontremos
PlannerState LocalPlanner::setSpeed() {
  double dist = distancia(pos, posGoal);

  ROS_INFO("Distancia: %f",dist);

  if (dist <= TOLERANCIA) {
    state = goal_achieved;
    v_angular = 0;
    v_lineal = 0;
  } else {
    //Rellenar y enviar Twist.
    geometry_msgs::Twist mensajeTwist;
    // La velocidad es cero si estamos girando mucho
    mensajeTwist.linear.x = (state != turning) * v_lineal;
    mensajeTwist.angular.z = v_angular;
    commandPub.publish(mensajeTwist);
  }

  return state;
}
