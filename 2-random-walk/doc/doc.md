---
title: "Técnicas de los Sistemas Inteligentes"
subtitle: "Práctica 1 - Entrega 2: Navegación local con y sin mapa"
author:
  - José Pimentel Mesones
  - Lothar Soto Palma
  - Francisco David Charte Luque
  - José Carlos Entrena Jiménez
toc: true
toc-depth: 2
numbersections: true
lang: spanish
polyglossia-lang.name: spanish
mainfont: Droid Serif
monofont: Source Code Pro

geometry: "a4paper, top=2.5cm, bottom=2.5cm, left=3cm, right=3cm"
abstract:
  Las técnicas basadas en navegación local pretenden guiar a un agente a lo largo de un mapa de forma que la trayectoria que sigue no está predefinida, es decir, se toman decisiones locales. En esta práctica se ha completado una implementación de un agente reactivo con navegación mediante campos de potencial con un mayor ángulo de visión, campos atractivos y repulsivos y detección de mínimos locales. Asimismo, se ha añadido cierta funcionalidad al agente, cono una gestión de la huida de mínimos locales mediante una selección heurística de objetivos secundarios atendiendo a los mapas de coste.
header-includes:
  - \renewcommand{\thesubsection}{\thesection.\alph{subsection}}
---

\pagebreak

# Tareas para mejorar el comportamiento reactivo del robot respecto al uso de campos de potencial (navegación local sin mapa)

## Conseguir aumentar el campo de visión
  **El campo de visión del robot simulado es de 270º, ¿cómo hacer para que el escaneo laser sea mayor que el actualmente implementado?**

  En el archivo _myPlannerLite.h_ hemos cambiado el valor de las constantes `MIN_SCAN_ANGLE_RAD` y `MAX_SCAN_ANGLE_RAD` a `-135.0/180*M_PI` y `135.0/180*M_PI` respectivamente, para aumentar el ángulo de visión a un total de 270º.

## Conseguir que no se demore tanto tiempo en detenerse cuando esté lo suficientemente cerca del objetivo
**¿Por qué tarda tanto en detenerse cuando está próximo al objetivo? ¿Cómo solucionarlo?**

  Cuando calculamos la componente atractiva y nos encontramos dentro del campo de atracción (representado por la componente _spread_), el módulo de la velocidad es inversamente proporcional a la distancia al objetivo, por lo que cuanto más nos acercamos a él, más lento se mueve el robot.

  Para solucionar este comportamiento, nuestra idea ha sido hacer constante el módulo de la componente atractiva de la velocidad cuando la distancia al objetivo es menor que la mitad del _spread_ del objetivo. Así, cuando estamos muy próximos al objetivo, la velocidad es constante y no se va reduciendo conforme nos acercamos, evitando que tarde tanto tiempo alcanzar el objetivo y detenerse.

  Tomamos esta velocidad constante como aquella obtenida cuando estamos a una distancia del objetivo de _spread_/2.

## Conseguir que el robot no tenga "comportamiento suicida", es decir, cuando está cerca de un obstáculo acelera y choca con él
**¿Cuál es la causa de este comportamiento suicida? ¿Cómo evitarlo?**

  Al encontrarnos muy próximos a un obstáculo, el cálculo de la componente repulsiva dará como resultado una velocidad lineal muy alta, consecuencia de querer alejarnos cuanto antes de dicho obstáculo. Sin embargo, como tratamos la velocidad lineal y la velocidad angular por separado, al aumentar la velocidad lineal el robot no tiene tiempo de girar antes de chocarse con el obstáculo, dando la sensación de que el robot se lanza hacia él.

  Como solución, hemos decidido hacer 0 la velocidad lineal del robot cuando se encuentra con un obstáculo y tiene que girar para evitarlo. De esta forma, permitimos al robot tomar la dirección deseada antes de volver a trasladarse, evitando así lanzarnos hacia el obstáculo. Cuando esto ocurra, tendremos al robot en un estado de giro, que al estar activado inhabilitará el paso de la velocidad lineal calculada, y en su lugar se pasará 0 como la componente `linear` del mensaje.

## Conseguir que, una vez que el robot se queda atrapado en una esquina (en un mínimo local), trate de salir de esta situación
\label{gestionminlocal}
**¿Cómo conseguirlo sin utilizar memoria (es decir, información de un mapa)?**

  En el archivo `myClientLite.cpp` se ha implementado una funcionalidad adicional en la función gestora de eventos de *feedback* que lleva un registro de la última posición y es capaz de analizar si el robot está quieto (o prácticamente quieto, mediante una tolerancia) y, tras transcurrir un tiempo razonable (que permite que el robot pivote sobre sí mismo si es necesario), asume que se ha encontrado un mínimo local.

  Cuando detectamos que el robot está atascado, cancelamos el goal actual y enviamos un nuevo goal, para intentar escapar de este mínimo local. Calculamos este nuevo goal utilizando el costmap, intentando dirigirnos hacia la zona que tenga menos obstáculos. En caso de no conseguir alcanzar este segundo goal, cancelamos la acción del robot. Si llegamos hasta él, restauramos el goal inicial.

## Conseguir suprimir las oscilaciones del robot
**¿Cuál es la causa de las oscilaciones? ¿Cómo eliminarlas en la mayor medida posible?**

  Hemos detectado oscilaciones en el robot cuando está muy cerca del objetivo. Esto se debe a que, al intentar encarar el objetivo, el mínimo giro posible hace que nos pasemos y tengamos que girar hacia el otro lado, volviendo a darse la misma situación. Para evitarlo, hemos aumentado la tolerancia en el ángulo del robot, subiendo el valor de la componente `EPSILON_ANGULAR`.

  Asimismo, se producen oscilaciones cuando el robot está bordeando una pared y el objetivo se encuentra al otro lado, ya que pretende girar hacia el objetivo pero para avanzar necesita hacerlo paralelamente, ya que la componente repulsiva le impide atravesar la pared. Para estos casos, se ha añadido una constante `MIN_LIN_SPEED_FOR_TURNS` que representa la velocidad lineal mínima que ha de tener el robot para permitirse girar. El valor de la constante se ha escogido experimentalmente, y se ha establecido a 0,05.

# Tareas para mejorar el comportamiento del robot mediante técnicas de navegación local con mapa.

##Contemplar el uso de distintos mapas para la experimentación
**Los mapas pueden generarse a partir de una imagen mediante el paquete mapserver. Ver explicación sobre manejo de mapas y mundos simulados en la documentación adjunta.**

  Incluímos capturas de pantalla con el uso de distintos mapas sobre los que hemos experimentado.


## Usar dos costmaps en el cliente
**Uno local configurado para tener una ventana activa que se desplace con el robot y otro global para tener una información sobre el costmap del mapa completo.**


## Usar el costmap local para poder encontrar una trayectoria local segura desde la pose actual
\label{heuristica}
**y que finalice en un punto (en el que no haya colisión) de la frontera del costmap lo más próximo posible al objetivo. Esto se llevará a cabo mediante un proceso de búsqueda heurística teniendo en cuenta las siguientes fuentes de información: la distancia al objetivo, costes de las celdas del costmap local local y muestras del escaneo láser.**

## Mejorar el comportamiento del cliente en los siguientes aspectos:
  - Detectar que el robot está "atascado" (demasiado tiempo en una región sin avanzar al objetivo), usando información de "feedback".
  - En caso de atasco cancelar goal actual.
  - Determinar un nuevo goal para sacarlo del atasco (usando el proceso de búsqueda local).
  - Enviar el nuevo goal, detectar que se ha alcanzado, y volver a enviar el goal original.

Como se explica en la sección \ref{gestionminlocal}, la función gestora de eventos de *feedback* analiza el tiempo que transcurre sin que el robot se traslade. Cuando se decide que se está ante un mínimo local, se cancela el objetivo primario. Una vez se ha cancelado, el cliente recurre a la heurística de *huida* que trata de buscar una trayectoria alternativa como se desarrolla en \ref{heuristica}. Se utiliza el método `waitForResult` del *action client* para esperar a que se alcance el objetivo secundario planeado. Si se alcanza sarisfactoriamente, se restablece el objetivo primario para intentar construir una nueva trayectoria hacia el mismo.


## Mejorar el comportamiento del servidor para ajustarse a los nuevos servicios que le va a requerir el cliente.

# Consideraciones adicionales.

## Condiciones de carrera

En la implementación del método `setTotalRepulsivo` en el archivo `myPlannerLite.cpp` se ha detectado un problema a la hora de acceder al vector que almacena los obstáculos que el robot está visualizando. Es posible que mientras se está calculando la componente repulsiva total, resultado de la suma de las componentes, se ejecute el callback `scanCallBack`, que cambia el vector de obstáculos, pudiendo modificarlo y derivar en un acceso a posiciones del vector que no están ocupadas, generando un _core_ y abortando el programa. Para solucionar esto, en el callback no generamos los nuevos elementos directamente sobre dicho vector, sino que rellenamos un vector nuevo con los obstáculos detectados y usamos el método `swap`, que los intercambia instantáneamente, impidiendo que se realicen accesos incorrectos.
