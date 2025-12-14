[![Review Assignment Due Date](https://classroom.github.com/assets/deadline-readme-button-22041afd0340ce965d47ae6ef1cefeee28c7c493a6346c4f15d667ab976d596c.svg)](https://classroom.github.com/a/eiNgq3fR)
[![Open in Visual Studio Code](https://classroom.github.com/assets/open-in-vscode-2e0aaae1b6195c2367325f4f02e2d04e9abb55f0b24a779b69b11b9e10269abc.svg)](https://classroom.github.com/online_ide?assignment_repo_id=22047721&assignment_repo_type=AssignmentRepo)
# Monitor continuo de signos vitales
<!-- Cambiar el titulo "Proyecto" por el nombre del proyecto -->

# Integrantes

- Juan Pablo Ruiz Sabogal
- Kevin Adrian Guerra 
<!-- Califico el informe unicamente a los integrantes que esten referenciados aqui y en el informe (PDF) -->

Indice:

1. [Descripción](#descripción)
2. [Informe](#informe)
3. [Implementación](#implementacion)
4. [Lista de anexos](#anexos)

## Descripción

El proyecto “Monitoreo Inteligente de Signos Vitales” consiste en el desarrollo de un sistema embebido basado en FPGA/SoC (Zynq-7000) orientado a monitorear en tiempo real tres parámetros fisiológicos clave: frecuencia cardíaca (BPM), saturación de oxígeno (SpO₂) y temperatura, con el fin de apoyar la detección temprana de anomalías en contextos donde no hay monitoreo continuo (por ejemplo, adultos mayores o zonas con menor acceso a servicios médicos). 


A nivel de hardware y arquitectura, el sistema integra un procesador ARM Cortex-A9 (PS) y lógica programable (PL), comunicados por bus AXI. El PS se encarga de la adquisición y control general, mientras que la PL implementa lógica específica para respuesta rápida y control de periféricos. El diseño se construye en Vivado (bloques y bitstream) y el software en C se desarrolla en Vitis. 


Para la adquisición de señales se emplean sensores biomédicos como MAX30102 (pulso y SpO₂) y MLX90614 (temperatura), además de una pantalla OLED para visualización local y un buzzer para alertas. La comunicación con sensores y pantalla se realiza por I2C, y los valores se calculan aplicando filtros/algoritmos sencillos para estabilizar lecturas; la pantalla se actualiza en tiempo real mediante un buffer gráfico. 

 
El sistema implementa un mecanismo de alertas que detecta condiciones fuera de rango y activa salidas (visual/sonora). En particular, se reporta una alarma sonora usando un AXI GPIO para controlar el buzzer cuando la frecuencia cardíaca supera un umbral (105 BPM). 


En cuanto al alcance logrado, el proyecto completó el procesamiento inicial de señales, la integración de alertas, y la visualización en OLED; la parte de Bluetooth (HC-05) quedó sin integración completa, por lo que la integración global se reporta como parcial debido a esa funcionalidad faltante. 


Finalmente, en las pruebas y validación se obtuvieron mediciones consistentes de BPM, SpO₂ y temperatura, con visualización continua y activación adecuada de alertas; se describe un funcionamiento estable del flujo hardware-software durante pruebas físicas. 


## Informe

<!-- Link que permita acceder al Informe, el cual debe estar subido a este repositorio -->

## Implementación

<!-- Video explicativo del funcionamiento del proytecto -->

## Archivos
<!-- CREAR UN DIRECTORIO CON EL NOMBRE "src" DONDE INVLUYAN LAS FUENTE (.c Y .h) QUE CREARON PARA EL PROOYECTO-->

<!-- NO OLVIDAD SUBIR EL PDF GENERADOR EN DEL BLOCK DESIGN-->
