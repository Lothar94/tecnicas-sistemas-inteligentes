---
title: "Técnicas de los Sistemas Inteligentes"
subtitle: "Práctica 1 - Entrega 3: Navegación global"
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
  En computación, el algoritmo A* es un proceso de búsqueda entre nodos de un grafo, que usa una heurística de estimación del coste hasta la solución, una función usualmente llamada $h(n)$, y una función de coste, $g(n)$, cuya suma nos da una estimación del coste de llegar de un nodo a una solución del problema. En esta práctica se ha completado una implementación del algoritmo A* con el uso de una cola con prioridad para la gestión de la lista de nodos abiertos, y lo hemos probado en distintos mundos para comprobar su efectividad.


header-includes:
  - \renewcommand{\thesubsection}{\thesection.\alph{subsection}}
---

\pagebreak

# Extensión del algoritmo A*

## Modificación de estructuras de datos

Para conseguir un funcionamiento más intuitivo del algoritmo A*, hemos hecho algunos cambios en las estructuras de datos ya presentes en el código. Hemos pasado de gestionar la lista de ABIERTOS como una lista de la STL a una cola con prioridad, que nos permite insertar ordenadamente los nodos en ABIERTOS según el valor de la función $f(n)$, lo que hace que tomar el mejor nodo sea rápido y no implique un proceso de búsqueda en toda la lista.

Dicha cola con prioridad va soportada sobre un vector de la STL, y utiliza un operador de comparación para el orden. Este  operador se ha implementado para los elementos `coupleOfCells`, que ha pasado de ser un *struct* a una clase para defenir el método `operator>`, que simula el comportamiento que había implementado en la función `compareFCost`.

## Implementación

El resto de la implementación consiste en seguir los pasos del algoritmo A*. Comenzamos creando el _goal_ y el _start_, e insertamos el punto de inicio en la cola de ABIERTOS. Una vez tenemos los datos iniciales, empezamos el proceso del algoritmo: tomamos el primer elemento de la cola de ABIERTOS y lo insertamos en la lista de CERRADOS. Si el elemento no es el _goal_, lo quitamos de la cola de ABIERTOS y lo expandimos, obteniendo sus nodos vecinos mediante el uso de la función `findFreeNeighborCell`. Obtenidos los vecinos, consideramos únicamente aquellos que no se encuentran en la lista de CERRADOS, y comprobamos si están en la cola de ABIERTOS, actualizando su valor si fuera necesario. Una vez aquí, repetimos el proceso.

Hemos mantenido las funciones $g(n)$ y $h(n)$ que venían en el código, que usan la distancia euclídea.

# Mejora del algoritmo A*
\label{mejoras}

En la experimentación hemos detectado que al pasarle un objetivo lejano al robot, el número de iteraciones no era suficiente para que el algoritmo A* encontrase un camino hasta dicho objetivo, lo que hacía que tuviera un comportamiento errático no deseado. Para solucionar esta situación, hemos hecho diversas modificaciones.

## Modificación del número de iteraciones

Aumentar el número de iteraciones máximas que el algoritmo puede realizar, lo que nos permite llegar a objetivos más lejanos, en el caso en el que la exploración en A* sea muy costosa.

## Adición de pesos

Para añadir un peso a la función $h(n)$, usamos una nueva función $w(n)$ que actúa como peso dinámico de la función $h(n)$. Para calcular dicho peso, dividimos el valor de la función $h$ calculada en el nodo actual entre el valor en el nodo _start_. De esta forma, conseguimos que la función $h$ pierda relevancia conforme nos vamos acercando al objetivo, tomando más importancia la distancia real hasta el punto (la función $g$) que la estimación que calculamos.

## Gestión de la resolución

Se ha reducido la resolución del mapa, para tener un menor número de nodos que explorar. Al tener una resolución demasiado alta, el número de casillas exploradas puede ser excesivo en comparación con la distancia real recorrida, lo que nos lleva a tener unos tiempos de computación altos.

## Cálculo del coste del *footprint*

Para evitar que el robot pase demasiado cerca de obstáculos e incluso se choque con ellos, se ha completado la implementación de la función `footprintCost`, que permite averiguar si la posición de la silueta del robot sobre una celda es legal (es decir, si no se choca con un obstáculo).

# Experimentación en Stage

## willow_garage

El mapa *willow_garage* es complejo y está lleno de obstáculos, por eso se ha elegido un camino corto para ilustrar el comportamiento del algoritmo A* en nuestro robot. En una de las "habitaciones" inferiores, se ha colocado al robot en un extremo y se le ha ubicado el objetivo en la otra punta de la habitación, como se observa en la Figura \ref{willow1} que representa el camino (sobre RViz) que encuentra el robot.

![\label{willow1} Camino seguido por el robot en *willow_garage*](img/willow_1.png)

Observamos en la Figura \ref{willow2} cómo las mejoras de la sección \ref{mejoras} permiten que el cálculo se haya realizado de forma rápida (aproximadamente 0,8 segundos) y el robot deje algo de espacio con los obstáculos.

![\label{willow2} El robot sortea los obstáculos del camino](img/willow_2.png)

Por último, se muestra en la Figura \ref{willow3} que el robot alcanza su objetivo con éxito.

![\label{willow3} El robot alcanza el objetivo](img/willow_3.png)
