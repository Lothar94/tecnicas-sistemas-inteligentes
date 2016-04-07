#include <ros/ros.h>
#include <move_base_msgs/MoveBaseAction.h>
#include <actionlib/client/simple_action_client.h>
#include "tf/transform_listener.h"
#include "costmap_2d/costmap_2d_ros.h"
#include <costmap_2d/costmap_2d.h>
#include <boost/thread.hpp>

typedef actionlib::SimpleActionClient<move_base_msgs::MoveBaseAction> MoveBaseClient;

struct Tupla {double x; double y;};

bool cancelar = false;
bool terminar = false;

void spinThread(){
  ros::spin();
}

//imprime en pantalla el contendio de un costmap
void printCostmap (costmap_2d::Costmap2DROS * costmap_ros) {
  //typedef basic_string<unsigned char> ustring;

  costmap_2d::Costmap2D *costmap;
  costmap = costmap_ros-> getCostmap();

  for (int x=0 ;x < costmap->getSizeInCellsX() ;x++){
    for (int y= 0; y < costmap->getSizeInCellsY();y++){
        std::cout << (int) costmap->getCost(x,y) << " ";
      }
      std::cout << std::endl;
    }
}

std::vector <unsigned int> findFreeNeighborCell (unsigned int CellID, costmap_2d::Costmap2D *costmap){
  unsigned int mx, my;
  //convertir el índice de celda en coordenadas del costmap
  costmap->indexToCells(CellID,mx,my);
  ROS_INFO("CellID = %d, mx = %d, my= %d", CellID, mx, my);
  std::vector <unsigned int>  freeNeighborCells;

  for (int x=-1;x<=1;x++)
    for (int y=-1; y<=1;y++){
    //comprobar si el índice es válido
    if ((mx+x>=0)&&(mx+x < costmap->getSizeInCellsX())&&(my+y >=0 )&&(my+y < costmap->getSizeInCellsY())){
      ROS_WARN("Index valid!!!!! costmap[%d,%d] = %d", mx+x,my+y,costmap->getCost(mx,my));
      if(costmap->getCost(mx+x,my+y) == costmap_2d::FREE_SPACE   && (!(x==0 && y==0))){
        unsigned int index = costmap->getIndex(mx+x,my+y);
        ROS_INFO("FREECELL: %d con valor %d", index, costmap_2d::FREE_SPACE);
        freeNeighborCells.push_back(index);
      }
      else {
        unsigned int index = costmap->getIndex(mx+x,my+y);
        ROS_INFO("OCCUPIEDCELL: %d con valor %d", index, costmap->getCost(mx+x,my+y));
      }
    }
  }
  return  freeNeighborCells;
}

//función para rellenar una PoseStamped a partir de las coordenadas del mundo obtenidas del costmap.
void rellenaPoseStamped (double wx, double wy, geometry_msgs::PoseStamped &pose, costmap_2d::Costmap2DROS *costmap){
    pose.header.stamp =  ros::Time::now();
    //el marco de coordendas de la posición debe ser el mismo frame que el del goal y este, a su vez, el mismo que el del costmap, por defecto debe tenerlo
    pose.header.frame_id = costmap->getGlobalFrameID();
    pose.pose.position.x = wx;
    pose.pose.position.y = wy;
    pose.pose.position.z = 0.0;
    pose.pose.orientation.x = 0.0; //la orientación en principio no interesa.
    pose.pose.orientation.y = 0.0;
    pose.pose.orientation.z = 0.0;
    pose.pose.orientation.w = 1.0;
}

bool localPlanner(const geometry_msgs::PoseStamped& start, const geometry_msgs::PoseStamped& goal, std::vector<geometry_msgs::PoseStamped>& plan, costmap_2d::Costmap2DROS *costmap_ros){
    ROS_INFO("localPlanner: Got a start: %.2f, %.2f, and a goal: %.2f, %.2f", start.pose.position.x, start.pose.position.y, goal.pose.position.x, goal.pose.position.y);

    plan.clear();
    costmap_2d::Costmap2D *costmap = costmap_ros->getCostmap();

    //pasamos el goal y start a coordenadas del costmap
    double goal_x = goal.pose.position.x;
    double goal_y = goal.pose.position.y;
    unsigned int mgoal_x, mgoal_y;
    //transformamos las coordenadas del mundo a coordenadas del costmap
    costmap->worldToMap(goal_x,goal_y,mgoal_x, mgoal_y);
    if ((mgoal_x>=0)&&(mgoal_x < costmap->getSizeInCellsX())&&(mgoal_y>=0 )&&(mgoal_y < costmap->getSizeInCellsY()))
      unsigned int indice_goal = costmap->getIndex(mgoal_x, mgoal_y);
    else ROS_WARN("He recibido unas coordenadas del mundo para el objetivo que está fuera de las dimensiones del costmap");

    //transformamos las coordenadas de la posición inicial a coordenadas del costmap.
    double start_x = start.pose.position.x;
    double start_y = start.pose.position.y;
    unsigned int mstart_x, mstart_y;
    costmap->worldToMap(start_x,start_y, mstart_x, mstart_y);
    unsigned int start_index = costmap->getIndex(mstart_x, mstart_y);


    //*************************************************************************
    // ya tenemos transformadas las coordenadas del mundo a las del costmap,
    // ahora hay que implementar la búsqueda de la trayectoria, a partir de aquí.
    //*************************************************************************
}

//Proporciona información mientras se intenta alcanzar la meta.
void feedbackCBGoal0( const move_base_msgs::MoveBaseFeedback::ConstPtr& feedback ){
  Tupla posicion;

  static double tiempo_inicial;
  static bool primera = true;
  static bool primera_igual = true;

  const double tiempo_maximo = 3.0;
  posicion.x = feedback->base_position.pose.position.x;
  posicion.y = feedback->base_position.pose.position.y;

  static Tupla pos_inicial = {posicion.x, posicion.y};

  if (cancelar) {
    ROS_INFO("cancelar false");
    cancelar = false;
    pos_inicial.x = posicion.x;
    pos_inicial.y = posicion.y;
  } else if (posicion.x == pos_inicial.x && posicion.y == pos_inicial.y) {
    if (primera_igual) {
      tiempo_inicial = ros::Time::now().toSec();
      ROS_INFO("tiempo_inicial: %f", tiempo_inicial);
      primera_igual = false;
    } else if (ros::Time::now().toSec() - tiempo_inicial >= tiempo_maximo) {
      cancelar = true;
      ROS_INFO("cancelando a los %f segundos", ros::Time::now().toSec() - tiempo_inicial);
    }
  } else {
    primera_igual = true;
    pos_inicial.x = posicion.x;
    pos_inicial.y = posicion.y;
  }
}

//Detecta si se alcanzó la meta o se canceló.
void doneCBGoal0( const actionlib::SimpleClientGoalState& state,
   const move_base_msgs::MoveBaseResultConstPtr& result ){
  if (state.state_ == actionlib::SimpleClientGoalState::SUCCEEDED){
    ROS_INFO("¡¡ Objetivo alcanzado !!");
    terminar = true;
  }
  else if (state.state_ == actionlib::SimpleClientGoalState::ABORTED){
    ROS_INFO("Objetivo abortado");
  }
  else if (state.state_ == actionlib::SimpleClientGoalState::REJECTED){
    ROS_INFO("Objetivo cancelado");
  }
}

//Indica del establecimiento del objetivo y que este está activo.
void activeCBGoal0(){
  ROS_INFO("El goal actual esta activo");
}

int main(int argc, char** argv){
  ros::init(argc,argv, "client_node");

  tf::TransformListener tf(ros::Duration(10));
  costmap_2d::Costmap2DROS* localcostmap = new costmap_2d::Costmap2DROS("local_costmap", tf);
  costmap_2d::Costmap2DROS* globalcostmap = new costmap_2d::Costmap2DROS("global_costmap", tf);

  //Pausamos hasta iniciar el servidor.
  localcostmap->pause();
  globalcostmap->pause();

  //crear el "action client"
  //true hace que el cliente haga "spin" en su propia hebra
  //Ojo: "move_base" puede ser cualquier string, pero hay que pensarlo como un "published topic". Por tanto, es importante que
  //El action server con el que va a comunicar tenga el mismo topic. E.d., al llamar a "as(n,string,....)" en el lado del
  //Action server hay que usar el mismo string que en el cliente.
  MoveBaseClient ac("mi_move_base", true);  //<- poner "mi_move_base" para hacer mi propio action server.

  //Creamos una hebra para actualizar los costmaps en background
  boost::thread spin_thread(&spinThread);

  //Esperar 60 sg. a que el action server esté levantado.
  ROS_INFO("Esperando a que el action server mi_move_base se levante");
  //Si no se conecta, a los 60 sg. se mata el nodo.
  ac.waitForServer(ros::Duration(60));

  ROS_INFO("Conectado al servidor mi_move_base");

  localcostmap->start();
  globalcostmap->start();
  printCostmap(localcostmap);
  costmap_2d::Costmap2D *micostmap = localcostmap->getCostmap();
  std::vector<unsigned int> vecinas = findFreeNeighborCell(1096,micostmap);

  // Read x, y and angle params
  ros::NodeHandle nh;

  double  x, y, theta, res;
  nh.getParam("goal_x", x);
  nh.getParam("goal_y", y);
  nh.getParam("goal_theta", theta);
  nh.getParam("local_costmap/resolution",res);

  ROS_INFO("La resolución del costmap local es %f", res);

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


  while(!terminar){


    if(cancelar){
      r.sleep();

      move_base_msgs::MoveBaseGoal nuevo_goal;
      nuevo_goal.target_pose.header.frame_id = 	"map";
      nuevo_goal.target_pose.header.stamp =	ros::Time::now();
      nuevo_goal.target_pose.pose.position.x = -8;//-18.174;
      nuevo_goal.target_pose.pose.position.y = 19;//	25.876;
      nuevo_goal.target_pose.pose.orientation.w = 1;//	1;

      ROS_INFO("El goal actual ha sido cancelado.");
      ac.cancelGoal();
      ac.waitForResult(ros::Duration(10.0));
      if (ac.getState() == actionlib::SimpleClientGoalState::PREEMPTED){
        ac.sendGoal(nuevo_goal);
        ROS_INFO("Esperando al resultado de la nueva acción.");
        ac.waitForResult();
        if (ac.getState() == actionlib::SimpleClientGoalState::SUCCEEDED){
          ac.sendGoal(goal, &doneCBGoal0, &activeCBGoal0, &feedbackCBGoal0);
          //Esperar al retorno de la acción
          ROS_INFO("Esperando al resultado  de la acción");
          cancelar = false;
        }
        else{
          ROS_INFO("Objetivo secundario fallido.");
          terminar = true;
        }
      }
    }
  }
  return 0;
}
