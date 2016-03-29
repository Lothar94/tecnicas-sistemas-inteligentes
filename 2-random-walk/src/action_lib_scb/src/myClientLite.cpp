#include <ros/ros.h>
#include <move_base_msgs/MoveBaseAction.h>
#include <actionlib/client/simple_action_client.h>

typedef actionlib::SimpleActionClient<move_base_msgs::MoveBaseAction> MoveBaseClient;

struct Tupla {double x; double y;};

Tupla pos_inicial;
double tiempo_inicial;
bool cancelar = false;
bool primera = true;
bool primera_igual = true;

double distancia(Tupla src, Tupla dst){
    return sqrt((src.x - dst.x) * (src.x - dst.x) +
                (src.y - dst.y) * (src.y - dst.y));
}

//Proporciona información mientras se intenta alcanzar la meta.
void feedbackCBGoal0( const move_base_msgs::MoveBaseFeedback::ConstPtr& feedback ){
  /*Tupla posicion;
  double dist;
  posicion.x = feedback->base_position.pose.position.x;
  posicion.y = feedback->base_position.pose.position.y;

  if(primera){
    pos_inicial.x = posicion.x;
    pos_inicial.y = posicion.y;
    primera = false;
  }
  else{
    dist = distancia(posicion, pos_inicial);
    ROS_INFO("Distancia: %f", dist);
    if(dist > 1.0){
      cancelar = true;
    }
  }*/
  double tiempo;
  Tupla posicion;
  static double tm = 0.0;
  double tiempo_maximo = 3.0;
  posicion.x = feedback->base_position.pose.position.x;
  posicion.y = feedback->base_position.pose.position.y;

  if( primera ){
    pos_inicial.x = posicion.x;
    pos_inicial.y = posicion.y;
    primera = false;
  }
  else{
    if( posicion.x == pos_inicial.x && posicion.y == pos_inicial.y ){
      if( primera_igual ){
        tiempo_inicial = ros::Time::now().toSec();
        ROS_INFO("tiempo_inicial: %f", tiempo_inicial);
        primera_igual = false;
      }
      else{
        if( tm < tiempo_maximo ){
          tiempo = ros::Time::now().toSec();
          tm = tiempo-tiempo_inicial;
          ROS_INFO("tiempo: %f", tm);
        }
        else{
          cancelar = true;
          ROS_INFO("cancelar");
        }
      }
    }
    else{
      pos_inicial.x = posicion.x;
      pos_inicial.y = posicion.y;
    }
  }
}

//Detecta si se alcanzó la meta o se canceló.
void doneCBGoal0( const actionlib::SimpleClientGoalState& state,
   const move_base_msgs::MoveBaseResultConstPtr& result ){
  if (state.state_ == actionlib::SimpleClientGoalState::SUCCEEDED){
    ROS_INFO("¡¡ Objetivo alcanzado !!");
  }
  else if (state.state_ == actionlib::SimpleClientGoalState::ABORTED){
    ROS_INFO("Objetivo abortado");
  }
  else if (state.state_ == actionlib::SimpleClientGoalState::REJECTED){
    ROS_INFO("Objetivo cancelado");
  }
  ros::shutdown();
}

//Indica del establecimiento del objetivo y que este está activo.
void activeCBGoal0(){
  ROS_INFO("El goal actual esta activo");
}

int main(int argc, char** argv){
  ros::init(argc,argv, "client_node");

  //crear el "action client"
  //true hace que el cliente haga "spin" en su propia hebra
  //Ojo: "move_base" puede ser cualquier string, pero hay que pensarlo como un "published topic". Por tanto, es importante que
  //El action server con el que va a comunicar tenga el mismo topic. E.d., al llamar a "as(n,string,....)" en el lado del
  //Action server hay que usar el mismo string que en el cliente.
  MoveBaseClient ac("mi_move_base", true);  //<- poner "mi_move_base" para hacer mi propio action server.

  //Esperar 60 sg. a que el action server esté levantado.
  ROS_INFO("Esperando a que el action server mi_move_base se levante");
  //Si no se conecta, a los 60 sg. se mata el nodo.
  ac.waitForServer(ros::Duration(60));

  ROS_INFO("Conectado al servidor mi_move_base");

  // Read x, y and angle params
  ros::NodeHandle nh;
  double  x, y, theta;
  nh.getParam("goal_x", x);
  nh.getParam("goal_y", y);
  nh.getParam("goal_theta", theta);

  //Enviar un objetivo a move_base
  move_base_msgs::MoveBaseGoal goal;
  std::cerr << "x,y,theta:" <<x<<y<<theta<< std::endl;
  goal.target_pose.header.frame_id = 	"map";
  goal.target_pose.header.stamp =	ros::Time::now();
  goal.target_pose.pose.position.x =-14;//-18.174;
  goal.target_pose.pose.position.y =23;//	25.876;
  goal.target_pose.pose.orientation.w =1;//	1;

  ROS_INFO("Enviando el objetivo");
  ac.sendGoal(goal, &doneCBGoal0, &activeCBGoal0, &feedbackCBGoal0);

  ros::Rate r(5); // 5 hz

  while(!cancelar){
    ros::spinOnce();
    r.sleep();
  }

  if(cancelar){
    ROS_INFO("El goal actual ha sido cancelado.");
    ac.cancelGoal();
    ac.waitForResult(ros::Duration(10.0));
  }

  //Esperar al retorno de la acción
  ROS_INFO("Esperando al resultado  de la acción");
  ac.waitForResult();

  return 0;
}
