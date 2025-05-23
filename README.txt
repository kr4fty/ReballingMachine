Librería max6675.h
==================
Se utiliza un fork de la librería de Adafruit, porque esta ultima NO utiliza 
SPI por hardware. En su lugar se utiliza la siguiente:
    https://github.com/banoz/MAX6675-library

    PINOUT
    ------
    SO:  MISO
    CS:  chip Select
    SCK: Entrada reloj
    VCC: 3.3V or 5V
    GND: GND

Librería Adafruit_ST7735.h
==========================
Se utiliza la librería de Adafruit
    https://github.com/adafruit/Adafruit-ST7735-Library

    PINOUT
    ------
    VLED:   Soporte LED UTFT
    SCL:    Entrada de reloj serie
    SDA/SDI:Entrada de datos serie
    DC:     Selección de datos / comandos
    RST:    Restablecer nivel bajo activo
    CS:     Selección de chip, nivel bajo activo
    GND:    Tierra
    VDD33:  Pin de fuente de alimentación 3.3V

Libreria PID_v1.h
=================
Librería PID
    https://github.com/br3ttb/Arduino-PID-Library

Librería AiEsp32RotaryEncoder.h
===============================
Librería para el control de un encoder y un botón
    https://github.com/kr4fty/ai-rotary-encoder

Librería ElegantOTA.h
=====================
Librería para la actualizacion del firmware utlizando OTA
    https://github.com/ayushsharma82/ElegantOTA

Librería ArduinoOTA.h (UTILIZADO)
=====================
Libreria para controlar las actualizaciones por medio de OTA
    https://github.com/arduino/arduinoOTA

/******************************************************************************
                               Corriente Alterna
******************************************************************************/

Para controlar la potencia de los calentadores conectados a la linea eléctrica
domiciliaria, 220Vca, se utilizara un triac, el cual me permitirá controlar el
angulo de fase y de esta forma manejar la potencia que le llega a los calenta-
dores.
El triac se activa con un pequeño pulso en el pata Gate (G) del triac, el cual
permanecerá activo hasta que el ciclo de la señal eléctrica se haga cero, con
una duración del pulso es de unos 10uSeg. Se debe enviar en cada cruce por cero 
para activarlo. Caso contrario solo permanecerá activo en el semiciclo
El angulo de fase lo controlaremos sabiendo cuando exactamente la señal pasa por
cero, enviando a partir de ese momento y a la fase deseada, el pulso de control

Control utilizado:
    https://github.com/pmarchini/Esp32Dimmer

Control de Cruce por Cero
=========================
https://www.reddit.com/r/AskElectronics/comments/vgd0lq/zerocrossing_detector_circuit_with_esp32/

    Problemas:
        * Lecturas/Interrupciones falsas por interferencia
            º Solución 1, usando un Smith Triger (use un 74LS244/14, con muy 
              buenos resultados. La contra es que se usa otro IC dedicado
            º Solución 2, usando filtro entes del puente (hubo pequeña mejora)
            º Solución 3, una vez recibido un pulso de interrupción, se ignora 
              en una pequeña ventana de tiempo las interrupciones y asi evitar
              falsos positivos. Buenos resultados
        * Utilizando millis y micros no se puede controlar correctamente el
          pulso para activar el TRIAC, debido a que posiblemente no llegamos
          con el tiempo a activarlo el pulso
            º Se utilizo interrupciones de dos timers, uno para la subida del
              pulso y el segundo para la bajada

Control de Fase
===============
    Problemas:
        * Al ver mediante el osciloscopio los pulsos enviados en cada cruce
          por cero, se ve que el ancho de dicho pulso es proporcional a el
          valor del capacitor de filtrado que esta antes del puente de diodos.
          Con un capacitor de 100nF me da un ancho aproximado de 1200 uSeg.
          Por lo que a el momento 0 desde donde comienza cada ciclo, esta en 
          aproximadamente 600 desde el momento en que se recibe la interrupción

        * Al trabajar con UNA salida pseudo-PWM utilizamos dos timers para
          controlar el pulso de activación del Triac.
          Ahora si quisiera trabajar con dos salidas al mismo, pseudo-PWM,
          con este método seria imposible, ya que al enviar un pequeño pulso
          los timers cuentan hasta:

              Timer 1: timerTicks <---------- Pulso de subida
              Timer 2: timerTicks + G_PULSE_WIDTH <-----Pulso de bajada
          
          El tema es como implementar los dos pulsos simultáneos, que lógica
          usar si se quiere implementar por medio de los dos timers, por que
          se solaparían

          Solución:
            Se utiliza un solo Timer. Este oscila a una frecuencia de 1.8MHz y
            se configura a timerAlarm() para que produzca un interrupción en 
            unos TIMER_INTERVAL cuentas. Este valor de TIMER_INTERVAL es fijo
            por lo que me dara los 10 mili segundos divididos en 180, de los
            grados. Luego dentro de cada interrupción voy sumando un contador y
            verificando en cada momento si una de las salidas coincide con la
            cuenta de este contador para poner en estado alto la salida.
            Luego para bajar, como TIMER_INTERVAL es de aprox. 55.55 uSeg, en
            la siguiente interrupción se pone en bajo el pin de salida,
            logrando de esta manera que se envie un pulso por la salida
            seleccionada al angulo de disparo requerido.
            Lo bueno de esta solicion es que puedo enviar en simultaneo, mismo
            y único Timer, varios pulsos hacia el Triac.
            Como contra se puede decir que consume mas recursos del MCU ya que
            el timer entrara a la ISR muchísima mas veces ya que el timer se
            desborda cada TIMER_INTERVAL, y esto es continuamente. Por este 
            motivo dicha ISR deberá esta MUY BIEN OPTIMIZADA


Calentadores
============
    * Calentador Inferior
          Se utiliza uno de 1300W, obtenido de una Air Fryer eléctrica
          39 Ohms, 14cm de diámetro aproximado

    * Calentador Superior
          Se utiliza un calentador de 1800W de una Pava eléctrica
          27 Ohms, 8cm de diámetro aproximado

TIMER y PWM:
============
    Problema:
        Se había implementado mediante el Timer1 y Timer2 una reimplementación
        de un PWM, pero adaptado para nuestro uso. Su función era la siguiente:
          Espero por la llega de un cruce por cero y de acuerdo con el dutty
          seleccionado, envió un pulso (HIGH), esto medido con el timer1. Luego
          al activarse el timer2 bajo el pin a LOW.
          En resumen: al momento de la llegada del cruce por cero, se espera un
          tiempo, duty, y se envía un pulso, de 15uS, para activar el TRIAC.
        Esto se había implementado con la API v2 del Timer, pero ahora se
        actualizo a la v3. Se re acomodo de acuerdo a los cambios de la nueva v3
        pero aun no se probo en placa.

PERFILES DE TEMPERATURA
=======================
    Se Intentara leer archivos de configuración, por ejemplo en formato JSON,
    que contenga el perfil a utilizar. De este modo se buscara tener varios de
    estos perfiles para ser seleccionados en el momento de arranque.
    
    Ejemplo de archivos:
     https://github.com/dukeduck1984/uReflowOven-Esp32-Micropython/tree/master/MAIN/profiles
    
    Solución implementada:
        Por ahora se guardan Perfiles en my_profiles.h y se cargan desde ahi

    

///////////////////////////////////////////////////////////////////////////////
HACER
=====
    * Utilizar interrupciones del Timer para recrear el pulso G hacia el Pin
      de activación del TRIAC
      
        Solución 1: Se utiliza DigitalWrite, ya que esta muy bien implementada
                    y es muy rápida. Nada que ver con la implementación hecha
                    para los ATMEGA328 (Arduino UNO).
        Solución 2: Se implemento interrupciones de los timers con resultados
                    positivos. Lo único es que el ancho del pulso generado no
                    tiene un ancho fijo. Aparentemente por ruido????

    * Calculo de superficie del semiciclo en función del angulo de disparo
      Al no ser una curva escalón la superficie/potencia entregada no va a ser
      lineal a medida que nos movemos a través del semiciclo con el pulso en G.
      Por tal, me imagino que tendré que hacer una variable vector con valores
      de los ángulos de fase y sus equivalentes en potencia

      Otra solución seria trabajar con medio semiciclo por vez, de 0 a 90º y de
      90º a 180º

      Solución:
                Se obtuvo la ecuación real de la superficie correspondiente al
                valor seteado del dutty. Para que quede lineal desde 0 a 180º.
                
                = acos(1-encoderCounter/90.0)*(180.0/M_PI)
    
    * Implementar una función genérica que se encargue de actualizar los datos
      a mostrar en el LCD. 
      Este debería ocupar menos tiempo que imprimir nuevos datos a medida que
      se van actualizando

      Solución:
                Implementado
    
    * Reordenar y dividir el código en diferentes archivos/headers, para lograr
      mas legibilidad

      Solución:
                Implementado
    
    * Utilizar Librería Pthreads para hacer un control en paralelo de la
      temperatura de los calentadores
    
    * Tratar de utilizar PWM en modo 2 (asi se llama en STM32, cuando el pulso)
      recién pasa a nivel alto, cuando se desborda el contador. A la inversa 
      que el PWM tradicional. AVERIGUAR ESTO!!!

///////////////////////////////////////////////////////////////////////////////