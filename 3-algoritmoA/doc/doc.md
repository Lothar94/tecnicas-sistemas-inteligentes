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

Asimismo, la función de orden que hemos usado para la cola con prioridad estaba ya implementada como el método _compareFCost_, pero para simplificar, hemos convertido el tipo de dato coupleOfCells de _struct_ a _class_, y hemos implementado un operador de comparación. 

# Mejora del algoritmo A*

# Experimentación en Stage
