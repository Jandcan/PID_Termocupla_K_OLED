# PID_Termocupla_K_OLED
Video de ejemplo: https://youtu.be/l73wgO0hXsQ
El formato de la trama de envio al Graficador Serial es: "$$d[dato]#d[dato]#...d[dato]#QQ\r\n", remplazar lo que esta en parentesis
Para la instalacion del Graficador Serial:
1) Abrir Matlab
2) Ir a la pestaña Apps
3) Instalar el archivo que se encuentra en la carpeta "Matlab"

Pasos para ejecutar el programa:
1) Clonar el repositorio
2) Descargar un emulador de puertos COM, por ejemplo: http://www.eterlogic.com/Products.VSPE.html
3) Crear un puerto COM virtual con el emulador.
4) Abrir el archivo de la carpeta "PROTEUS" (debes tener Proteus 8.8)
5) Doble clic en el Atmega328P y especificar a ruta del archivo .Hex O .elf, esto se encuentra en la carpeta "Debug"
6) Ejecutar.

