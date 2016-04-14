/*********************************************************************
*
* Software License Agreement (BSD License)
*
*  Copyright (c) 2008, Willow Garage, Inc.
*  Copyright (c) 2015, Juan Fdez-Olivares
*  All rights reserved.
*
*  Redistribution and use in source and binary forms, with or without
*  modification, are permitted provided that the following conditions
*  are met:
*
*   * Redistributions of source code must retain the above copyright
*     notice, this list of conditions and the following disclaimer.
*   * Redistributions in binary form must reproduce the above
*     copyright notice, this list of conditions and the following
*     disclaimer in the documentation and/or other materials provided
*     with the distribution.
*   * Neither the name of Willow Garage, Inc. nor the names of its
*     contributors may be used to endorse or promote products derived
*     from this software without specific prior written permission.
*
*  THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
*  "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
*  LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
*  FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
*  COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
*  INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
*  BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
*  LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
*  CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
*  LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
*  ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
*  POSSIBILITY OF SUCH DAMAGE.
*
* Authors: Juan Fdez-Olivares, Eitan Marder-Eppstein, Sachin Chitta
*********************************************************************/
#include "../include/my_astar_planner/myAstarPlanner.h"
#include <pluginlib/class_list_macros.h>

#include <algorithm>

// para debugging
#include <sstream>
#include <string>

// Hack para acceder al container de la priority_queue
template <class T, class S, class C>
S& Container(std::priority_queue<T, S, C>& q) {
  struct HackedQueue : private std::priority_queue<T, S, C> {
    static S& Container(std::priority_queue<T, S, C>& q) {
      return q.*&HackedQueue::c;
    }
  };
  return HackedQueue::Container(q);
}

//register this planner as a BaseGlobalPlanner plugin
PLUGINLIB_EXPORT_CLASS(myastar_planner::MyastarPlanner, nav_core::BaseGlobalPlanner)

namespace myastar_planner {

  //devuelve un puntero a un nodo en una lista de nodos (nodo = coupleOfCells) a partir del índice del nodo
  std::list<coupleOfCells>::iterator getPositionInList(std::list<coupleOfCells> & list1, unsigned int cellID);

  //comprueba si un índice de nodo existe en una lista de nodos.
  bool isContains(std::list<coupleOfCells> & list1, int cellID);

  MyastarPlanner::MyastarPlanner()
  : costmap_ros_(NULL), initialized_(false) {}

  MyastarPlanner::MyastarPlanner(std::string name, costmap_2d::Costmap2DROS* costmap_ros)
  : MyastarPlanner() {
    initialize(name, costmap_ros);
  }

  //inicializador del global_planner, mejor no tocar nada.
  void MyastarPlanner::initialize(std::string name, costmap_2d::Costmap2DROS* costmap_ros){
    if(!initialized_){
      costmap_ros_ = costmap_ros;
      costmap_ = costmap_ros_->getCostmap();

      ros::NodeHandle private_nh("~/" + name);

      //vamos a asumir estos parámetros, que no es necesario enviar desde el launch.
      private_nh.param("step_size", step_size_, costmap_->getResolution());
      private_nh.param("min_dist_from_robot", min_dist_from_robot_, 0.10);
      //world_model_ = new base_local_planner::CostmapModel(*costmap_);

      //el plan se va a publicar en el topic "planTotal"
      plan_pub_ = private_nh.advertise<nav_msgs::Path>("planTotal",1);

      initialized_ = true;
    }
    else
      ROS_WARN("This planner has already been initialized... doing nothing");
  }

  //esta función puede usarse para ayudar a calcular rutas seguras
  //está preparada para obtener el footprint del robot y devolver un valor representando el coste de la posición del robot.
  double MyastarPlanner::footprintCost(double x_i, double y_i, double theta_i){
    if(!initialized_){
      ROS_ERROR("The planner has not been initialized, please call initialize() to use the planner");
      return -1.0;
    }

    std::vector<geometry_msgs::Point> footprint = costmap_ros_->getRobotFootprint();
    //if we have no footprint... do nothing
    if(footprint.size() < 3)
      return -1.0;

    return -1.0;
  }

  //función llamada por move_base para obtener el plan.
  bool MyastarPlanner::makePlan(const geometry_msgs::PoseStamped& start,
    const geometry_msgs::PoseStamped& goal, std::vector<geometry_msgs::PoseStamped>& plan){

    //***********************************************************
    // Inicio de gestion de ROS, mejor no tocar nada en esta parte
    //***********************************************************
    if(!initialized_){
      ROS_ERROR("makePlan: The astar planner has not been initialized, please call initialize() to use the planner");
      return false;
    }

    ROS_INFO("MyastarPlanner: Got a start: %.2f, %.2f, and a goal: %.2f, %.2f", start.pose.position.x, start.pose.position.y, goal.pose.position.x, goal.pose.position.y);

    plan.clear();

    //obtenemos el costmap global  que está publicado por move_base.
    costmap_ = costmap_ros_->getCostmap();


    //Obligamos a que el marco de coordenadas del goal enviado y del costmap sea el mismo.
    //esto es importante para evitar errores de transformaciones de coordenadas.
    if(goal.header.frame_id != costmap_ros_->getGlobalFrameID()){
      ROS_ERROR("This planner as configured will only accept goals in the %s frame, but a goal was sent in the %s frame.",
          costmap_ros_->getGlobalFrameID().c_str(), goal.header.frame_id.c_str());
      return false;
    }

    tf::Stamped<tf::Pose> goal_tf;
    tf::Stamped<tf::Pose> start_tf;

    poseStampedMsgToTF(goal,goal_tf);
    poseStampedMsgToTF(start,start_tf);

    //obtenemos la orientación start y goal en start_yaw y goal_yaw.
    double useless_pitch, useless_roll, goal_yaw, start_yaw;
    start_tf.getBasis().getEulerYPR(start_yaw, useless_pitch, useless_roll);
    goal_tf.getBasis().getEulerYPR(goal_yaw, useless_pitch, useless_roll);


    /**************************************************************************/
    /*************** HASTA AQUÍ GESTIÓN DE ROS *********************************/
    /****************************************************************************/

    //pasamos el goal y start a un nodo (estructura coupleOfCells)
    coupleOfCells cpstart, cpgoal;
    double goal_x = goal.pose.position.x;
    double goal_y = goal.pose.position.y;
    unsigned int mgoal_x, mgoal_y;
    costmap_->worldToMap(goal_x,goal_y,mgoal_x, mgoal_y);
    cpgoal.index = MyastarPlanner::costmap_->getIndex(mgoal_x, mgoal_y);
    cpgoal.parent=0;
    cpgoal.gCost=0;
    cpgoal.hCost=0;
    cpgoal.fCost=0;

    double start_x = start.pose.position.x;
    double start_y = start.pose.position.y;
    unsigned int mstart_x, mstart_y;
    costmap_->worldToMap(start_x,start_y, mstart_x, mstart_y);
    cpstart.index = costmap_->getIndex(mstart_x, mstart_y);
    cpstart.parent = cpstart.index;
    cpstart.gCost = 0;
    cpstart.hCost = calculateHCost(cpstart.index,cpgoal.index);

    //insertamos el nodo inicial en abiertos
    openQueue.push(cpstart);

    ROS_INFO("Inserto en Abiertos: %d", cpstart.index );
    ROS_INFO("Index del goal: %d", cpgoal.index );

    unsigned int explorados = 0,
      currentIndex = cpstart.index;

    while (!(currentIndex == cpgoal.index || explorados >= 2000 || openQueue.empty())) {
      // escoger el nodo (coupleOfCells) de abiertos que tiene el valor más pequeño de f.
      coupleOfCells cOfCells = openQueue.top();
      currentIndex = cOfCells.index;

      // se inserta el nodo en cerrados (mediante constructor de copia)
      closedList.push_back(coupleOfCells(cOfCells));

      //ROS_INFO("Inserto en CERRADOS: %d", (*it).index );
      // ROS_INFO("G: %f, H: %f, F: %f", cOfCells.gCost, cOfCells.hCost, cOfCells.fCost);
      // ROS_INFO("Index: %d Parent: %d", cOfCells.index, cOfCells.parent);

      if (currentIndex != cpgoal.index) {
        //Si no hemos encontrado plan aún eliminamos el nodo insertado de ABIERTOS.
        openQueue.pop();

        //Buscamos en el costmap las celdas adyacentes a la actual
        std::vector<unsigned int> neighborCells = findFreeNeighborCell(currentIndex);

        //Ignoramos las celdas que ya existen en CERRADOS
        std::remove_if(neighborCells.begin(), neighborCells.end(), [&](unsigned int index) -> bool {
          return std::any_of(closedList.begin(), closedList.end(), [&index](coupleOfCells cell) -> bool {
            return cell.index == index;
          });
        });

        std::vector<coupleOfCells> &celdas_abiertos = Container(openQueue);

        //Determinamos las celdas que ya están en ABIERTOS y las que no están en ABIERTOS
        std::vector<unsigned int>::iterator fuera_begin = std::partition(neighborCells.begin(), neighborCells.end(), [&](unsigned int index) -> bool {
          return std::any_of(celdas_abiertos.begin(), celdas_abiertos.end(), [&index](coupleOfCells& cell) -> bool {
            return cell.index == index;
          });
        });

        //Añadimos a ABIERTOS las celdas que todavía no están en ABIERTO, marcando el nodo actual como su padre
        add_neighbors_to_queue(openQueue, fuera_begin, neighborCells.end(), currentIndex, cpstart.gCost, cpgoal.index);
        explorados++;

        //Para los nodos que ya están en abiertos, comprobar en cerrados su coste y actualizarlo si fuera necesario
        std::for_each(celdas_abiertos.begin(), celdas_abiertos.end(), [&](coupleOfCells& cell) {
          std::list<coupleOfCells>::iterator cerrada = std::find_if(closedList.begin(), closedList.end(), [&cell](coupleOfCells& closed_cell) {
            return closed_cell.index == cell.index;
          });
          (*cerrada).gCost = cell.gCost;
          (*cerrada).hCost = cell.hCost;
          (*cerrada).fCost = cell.fCost;
        });
      }
    }

    if (openQueue.empty()) {  // if the openList is empty: then failure to find a path
      ROS_INFO("Failure to find a path !");
    } else {
      //el plan lo construimos partiendo del goal, del parent del goal y saltando en cerrados "de parent en parent"
      //y vamos insertando al final los waypoints (los nodos de cerrados)

      ROS_INFO("PLAN ENCONTRADO !!!");

      //convertimos goal a poseStamped nueva

      geometry_msgs::PoseStamped pose;
      pose.header.stamp =  ros::Time::now();
      pose.header.frame_id = goal.header.frame_id;//debe tener el mismo frame que el goal pasado por parámetro
      pose.pose.position.x = goal_x;
      pose.pose.position.y = goal_y;
      pose.pose.position.z = 0.0;
      pose.pose.orientation.x = 0.0;
      pose.pose.orientation.y = 0.0;
      pose.pose.orientation.z = 0.0;
      pose.pose.orientation.w = 1.0;

      // Lo añadimos al plan
      plan.push_back(pose);
      ROS_INFO("Inserta en Plan: %f, %f", pose.pose.position.x, pose.pose.position.y);

      std::list<coupleOfCells>::iterator goal_it = std::find_if(closedList.begin(), closedList.end(), [&](coupleOfCells cell) { return cell.index == cpgoal.index; });
      coupleOfCells currentCouple = *goal_it;
      unsigned int currentParent = currentCouple.parent;

      while (currentCouple.index != currentParent) { //e.d. mientras no lleguemos al nodo start
        //encontramos la posición de currentParent en cerrados

        std::list<coupleOfCells>::iterator it = getPositionInList(closedList, currentParent);

        //hacemos esa posición que sea el currentCouple
        currentCouple.index = currentParent;
        currentCouple.parent = (*it).parent;
        currentCouple.gCost = (*it).gCost;
        currentCouple.hCost = (*it).hCost;
        currentCouple.fCost = (*it).fCost;

        //creamos una PoseStamped con la información de currentCouple.index

        //primero hay que convertir el currentCouple.index a world coordinates
        unsigned int mpose_x, mpose_y;
        double wpose_x, wpose_y;

        costmap_->indexToCells(currentCouple.index, mpose_x, mpose_y);
        costmap_->mapToWorld(mpose_x, mpose_y, wpose_x, wpose_y);

        //después creamos la pose
        geometry_msgs::PoseStamped pose;
        pose.header.stamp =  ros::Time::now();
        pose.header.frame_id = goal.header.frame_id;//debe tener el mismo frame que el de la entrada
        pose.pose.position.x = wpose_x;
        pose.pose.position.y = wpose_y;
        pose.pose.position.z = 0.0;
        pose.pose.orientation.x = 0.0;
        pose.pose.orientation.y = 0.0;
        pose.pose.orientation.z = 0.0;
        pose.pose.orientation.w = 1.0;

        //insertamos la pose en el plan
        plan.push_back(pose);
        ROS_INFO("Inserta en Plan: %f, %f", pose.pose.position.x, pose.pose.position.y);
        //hacemos que currentParent sea el parent de currentCouple
        currentParent = currentCouple.parent;
      }

      std::reverse(plan.begin(), plan.end());
      ROS_INFO("Vamos a publicar el plan.");

      //lo publica en el topic "planTotal"
      publishPlan(plan);
    }

    return openQueue.empty();
  };


  //calculamos H como la distancia euclídea hasta el goal
  double MyastarPlanner::calculateHCost(unsigned int start, unsigned int goal) {
    unsigned int mstart_x, mstart_y, mgoal_x, mgoal_y;
    double wstart_x, wstart_y, wgoal_x, wgoal_y;

    //trasformamos el indice de celdas a coordenadas del mundo.
    //ver http://docs.ros.org/indigo/api/costmap_2d/html/classcostmap__2d_1_1Costmap2D.html

    costmap_->indexToCells(start, mstart_x, mstart_y);
    costmap_->mapToWorld(mstart_x, mstart_y, wstart_x, wstart_y);
    costmap_->indexToCells(goal, mgoal_x, mgoal_y);
    costmap_->mapToWorld(mgoal_x, mgoal_y, wgoal_x, wgoal_y);

    return sqrt((pow(wstart_x - wgoal_x,2))+pow(wstart_y - wgoal_y, 2));
  }


  // Comparamos F para dos nodos.
  // Función de comparación para la cola con prioridad.
  bool MyastarPlanner::compareFCost(coupleOfCells const &c1, coupleOfCells const &c2){
     return c1.fCost < c2.fCost;
  }

  /*******************************************************************************/
  //Function Name: getPositnionInList
  //Inputs:the cellID, the list
  //Output: index of the cell in the list
  //Description: it is used to search the index of a cell in a list
  /*********************************************************************************/
  std::list<coupleOfCells>::iterator getPositionInList(std::list<coupleOfCells> & list1, unsigned int cellID){
     for (std::list<coupleOfCells>::iterator it = list1.begin(); it != list1.end(); it++){
       if (it->index == cellID)
           return it;
     }
  }


   /*******************************************************************************
   * Function Name: findFreeNeighborCell
    * Inputs: el índice de la celda
    * Output: a vector of free neighbor cells of the current cell
    * Description:it is used to find the free neighbors Cells of a the current Cell in the grid
    * findFreeNeighborCell(i) es el conjunto de celdas del costmap tales que
    *             costmap[x,y] = FREE_SPACE, donde (x,y) son las coordenadas en el costmap del índice i
    *
  *********************************************************************************/
  std::vector<unsigned int> MyastarPlanner::findFreeNeighborCell (unsigned int CellID){
    unsigned int mx, my;
    costmap_->indexToCells(CellID,mx,my);
    std::vector<unsigned int>  freeNeighborCells;

    for (int x=-1;x<=1;x++)
      for (int y=-1; y<=1;y++){
        //check whether the index is valid
       if ((mx+x>=0)&&(mx+x < costmap_->getSizeInCellsX())&&(my+y >=0 )&&(my+y < costmap_->getSizeInCellsY())){
          if(costmap_->getCost(mx+x,my+y) == costmap_2d::FREE_SPACE   && (!(x==0 && y==0))){
              unsigned int index = costmap_->getIndex(mx+x,my+y);
              freeNeighborCells.push_back(index);
          }
        }
    }
    return  freeNeighborCells;
  }


  /*******************************************************************************/
  //Function Name: isContains
  //Inputs: the list, the cellID
  //Output: true or false
  //Description: it is used to check if a cell exists in the open list or in the closed list
  /*********************************************************************************/
   bool isContains(std::list<coupleOfCells> & list1, int cellID){
     for (std::list<coupleOfCells>::iterator it = list1.begin(); it != list1.end(); it++){
       if (it->index == cellID)
           return true;
      }
     return false;
  }

  double MyastarPlanner::getMoveCost(unsigned int here, unsigned int there) {
    //calculo el coste de moverme entre celdas adyacentes como la distancia euclídea.
    return calculateHCost(here,there);
  }

  /*******************************************************************************/
  //Function Name: add_neighbors_to_queue
  //Inputs: the open queue, the neighbor Cells and the parent Cell
  //Output:
  //Description: it is used to add the neighbor Cells to the open queue
  /*********************************************************************************/
  template<class T, class S, class C>
  void MyastarPlanner::add_neighbors_to_queue(std::priority_queue<T, S, C>& abiertos, std::vector<unsigned int>::iterator first, std::vector<unsigned int>::iterator last, unsigned int parent, double parent_cost, unsigned int goal) {
    std::for_each(first, last, [&](unsigned int neighbor_cell) {
      T neighbor;

      neighbor.index = neighbor_cell;
      neighbor.parent = parent;
      //calculate the gCost
      neighbor.gCost = parent_cost + getMoveCost(parent, neighbor_cell);
      //calculate the hCost: Euclidian distance from the neighbor cell to the goalCell
      neighbor.hCost = calculateHCost(neighbor_cell, goal);
      //calculate fcost
      neighbor.fCost = neighbor.gCost + neighbor.hCost;

      abiertos.push(neighbor);
    });
  }

  //publicamos el plan para poder visualizarlo en rviz
  void MyastarPlanner::publishPlan(const std::vector<geometry_msgs::PoseStamped>& path){
    ROS_INFO("Longitud del path: %d", path.size());

    /*if (!initialized_) {
      ROS_ERROR("This planner has not been initialized yet, but it is being used, please call initialize() before use");
      return;
    }*/

    //create a message for the plan
    nav_msgs::Path gui_path;
    gui_path.poses.resize(path.size());

    if (!path.empty()){
      gui_path.header.frame_id = path[0].header.frame_id;
      gui_path.header.stamp = path[0].header.stamp;
    }

    // Extract the plan in world co-ordinates, we assume the path is all in the same frame
    for (unsigned int i = 0; i < path.size(); i++){
      gui_path.poses[i] = path[i];
    }

    plan_pub_.publish(gui_path);
    ROS_INFO("Se publicó el camino encontrado.");
  }

}
