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
  En computación, el algoritmo A* es un proceso de búsqueda entre nodos de un grafo, que usa una heurística de estimación del coste hasta la solución, una función usualmente llamada _h(n)_, y una función de coste, _g(n)_, cuya suma nos da una estimación del coste de llegar de un nodo a una solución del problema. En esta práctica se ha completado una implementación del algoritmo A* con el uso de una cola con prioridad para la gestión de la lista de nodos abiertos, y lo hemos probado en distintos mundos para comprobar su efectividad.


header-includes:
  - \renewcommand{\thesubsection}{\thesection.\alph{subsection}}
---

\pagebreak

# Extensión del algoritmo A*

## Modificación de estructuras de datos

Para conseguir un funcionamiento más intuitivo del algoritmo A*, hemos hecho algunos cambios en las estructuras de datos ya presentes en el código. Hemos pasado de gestionar la lista de ABIERTOS como una lista de la STL a una cola con prioridad, que nos permite ordenar los nodos en ABIERTOS según el valor de la función _f(n)_, lo que hace que tomar el mejor nodo sea rápido y no implique un proceso de búsqueda en toda la lista.

Dicha cola con prioridad va montada sobre un vector de la STL, y utiliza un operador de comparación para el orden. Este  operador se ha implementado para los elementos _coupleOfCells_, que ha pasado de ser un struct a una clase para defenir el método _operator>_, que simula el comportamiento que había implementado en la función _compareFCost_.

## Implementación

El resto de la implementación consiste en seguir los pasos del algoritmo A*. Comenzamos creando el _goal_ y el _start_, e insertamos el punto de inicio en la cola de ABIERTOS. Una vez tenemos los datos iniciales, empezamos el proceso del algoritmo: tomamos el primer elemento de la cola de ABIERTOS y lo insertamos en la lista de CERRADOS. Si el elemento no es el _goal_, lo quitamos de la cola de ABIERTOS y lo expandimos, obteniendo sus nodos vecinos mediante el uso de función _findFreeNeighborCell_. Obtenidos los vecinos, consideramos únicamente aquellos que no se encuentran en la lista de CERRADOS, y comprobamos si están en la cola de ABIERTOS, actualizando su valor si fuera necesario. Una vez aquí, repetimos el proceso.

Hemos mantenido las funciones _g(n)_ y _h(n)_ que venían en el código, que usan la distancia euclídea.

# Mejora del algoritmo A*

En la experimentación hemos detectado que al pasarle un objetivo lejano al robot, el número de iteraciones no era suficiente para que el algoritmo A* encontrase un camino hasta dicho objetivo, lo que hacía que tuviera un comportamiento errático no deseado. Para solucionar esta situación, hemos hecho diversas modificaciones:

- Aumentar el número de iteraciones máximas que el algoritmo puede realizar, lo que nos permite llegar a objetivos más lejanos, en el caso en el que la exploración en A* sea muy costosa.

- Añadir un peso a la función _h(n)_. Usamos una nueva función _w(n)_ que actúa como peso dinámico de la función _h(n)_. Para calcular dicho peso, dividimos el valor de la función _h_ calculada en el nodo actual entre el valor en el nodo _start_. De esta forma, conseguimos que la función _h_ pierda relevancia conforme nos vamos acercando al objetivo, tomando más importancia la distancia real hasta el punto (la función _g_) que la estimación que calculamos. 

- Reducir la resolución del mapa, para tener un menor número de nodos que explorar. Al tener una resolución demasiado alta, el número de casillas exploradas puede ser excesivo en comparación con la distancia real recorrida, lo que nos lleva a tener unos tiempos de computación altos.


# Experimentación en Stage
