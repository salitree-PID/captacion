//NODO 1 H2OLIVETREE. Coop. OLEAND. Puebla de Cazalla

// SDI-12 BUS_A CON 4 SENSORES DE METER (ALIM. 5V)
// 2 CAUDALIMETROS 4-20mA (ultrasonidos).
// 2 ELECTROVALVULAS

// placa ESP32 T8 V1.7

//15-09-23: se soluciona la reconexión wifi cuando hay pérdida de red de datos durante la ejecución del programa. Ya no es necesario resetear el micro cuando hay pérdida wifi.
//21-09-23: Se configura el caudalimetro de ultrasonidos sin filtro de amortiguacion para el cálculo de caudal. Menú M40 puesto a 0 seg.
//          Altura máxima deposito pasa de 2000 mm a 3000 mm
//          Ticker para grabar datos en SD cada 10 seg.
//          Asigna valores de error a las variables de las Hydros y la ES2 cuando hay error de lectura de sonda
//26-09-23: Se modifica la funcion de lectura de caudalimetro para mejorar su precisión. Se parte de Nodo3_Oleand_v0.ino. Pte. modificar valor de RBUCLE cuando se mida en campo
//02-10-23: Se modifica la posición de la condición de escritura en memoria SD (en version anterior estaba dentro del if(centinelaMuestra) y ahora se saca al loop principal)
//09-10-23: Se implementan valores de histeresis para conductividas y nivel máximo de llenado del depósito en la toma de decisión de cambio de válvulas, para evitar efectos rebote
//V5: 09-10-23: Se incluye nueva ES2 en salida del emisario hacia balsas debido a que en las balsas es muy complicado colocar sonda. La info a Thingspeak se transmite por el canal del Nodo3Oleand(balsas) y como
//              se envian solo 2 campos, se hace con la funcion "writeField()"
//    10-10-23: también se escribe en los campos 7 y 8 del canal 1 de Thingspeak y en campo 8 de canal 2, para no dejarlos al aire, ya que "writeFields()" escribe los 8 campos. ESTO SE QUITA

//v6: 19-10-23: se modifican las funciones "lecturasHydros21" y "lecturasES2" ya que devolvían valores negativos, pero su tipo de dato devuelto era "byte" que no reconoce valores negativos.
//               Se pone el valor devuelto como "int".
//     23-10-23: Se implementa nuevo método para determinar cuando hacer la captación de agua para riego (toma de muestras, calculo de media, histeresis)
//               se modifican los valores para apertura y cierre de EVs considerando que ahora EV1 será NA en lugar de NC (EV2 sigue siendo NC)
//               se escribe en el canal 2229411 de Thingspeak (campos 7 y 8), la conductividad y la temperatura de la ES2 del Emisario
//v7: 08-11-23: se incluye en la condición de captura de agua que haya caudal (además de prof y conductividad) ya que podría darse el caso de que la ES2 estuviera midiendo buena calidad de agua y se
//                abriese la EV2 (NC) indefinidamente hasta que arrancase la bomba del depósito intermedio (pudiéndose dañar el solenoide de dicha EV2)
//V8: 13-11-23: se implementa uso de telecomandos con Thingspeak. Se incluye tanto comandos, como configuracion de parámetros (las constantes involucradas pasan a ser variables).
//    16-11-23: se incluye en el código que los valores de variables de telecomandos se pongan a estado automático cuando hay pérdida de conexión Wifi. Los parámetros mantienen su valor y no se reinicializan
//              también se incluye que se escriban los valores de los parámetros configurables actuales en un nuevo canal de Thingspeak, para poder hacer comprobación de que se ha realizado el cambio en los mismos, así como
//              el valor del telecomando
//v8.1 13-12-23: se modifica el radio del deposito a 14.5 dm, max nivel deposito a 2200
//v9   19-12-23: se añade segundo bus SDI-12 (B) para conectar una de las ES2, ya que el conjunto 2 ES2 + 3 HYDROS-21 siguen dando problemas a pesar de haber cambiado la ES2 del deposito intermedio
//v9.1  20-12-23: no funcionan 2 SDI-12 en paralelo, por lo que hay que activar primero uno y luego otro por lo que se saca la inciacialización del setup() y se gestionan en loop()
//v10   08-01-24: se registra en SD y en ThingSpeak la conductividad media de la ES2 del depósito intermedio (tubería) que es la que controla el cierre y apertura de válvulas,
//               para lo que se pone la variable 'condMedia' como variable global en lugar de como variable local en la función 'evaluaCond()'.
//                tambien se corrige el error en reconexión WIFI "= !"WL_CONNECTED")" por "!= "WL_CONNECTED")"
//v11   23-02-24: se añade fichDatos.close(); al final del setup() para garantizar que el fichero queda cerrado tras un reset
//      26-02-24: se pone el tiempo de grabación en SD cada 30 seg. (antes cada 10 seg.)
//      08-03-24: se introduce cambio en la condicion del if() que controla si tener en cuenta la lectura de conductividad de tubería (deposito 0) en caso de valor de lectura erronea
//v12   21-03-24: se cambia el tipo de electroválvulas EV1 y EV2 pasando de tipo membrana a tipo bola. Por tanto se pasa de control por solenoide (alimentanto se opera EV y
//                no alimentando se deja de operar EV, por lo que hay que implementar cierta lógica dependiendo si la EV es NA o NC), a control motorizado (se alimenta un borne
//                para abrir EV y otro borne para cerrar EV y las EV son iguales, por lo que la lógica tiene otro planteamiento). Anteriormente el problema era el consumo
//                del relé y del solenoide y se evitaba una alimentación prolongada en situación de reposo y ahora el problema es el consumo del relé para el que también se debe
//                evitar una alimentación en reposo prolongada
//v13   01-04-24: también se implementa un nuevo bus SDI-12 (SDI12-3) para integrar 2 nuevas sondas: 1 Hydros-21 a arqueta4 y 1 ES2 a Emisario2 (zona norte).
//      06-05-24: se introducen los cambios de v11r1 y se hace lo mismo con SDI12-3, es decir, se añaden tiempos de establecimiento del bus.
//v14   07-05-24: se implementa nuevo control de hora para evitar errores de la misma cuando hay un reset y no hay wifi. En la función sincronizaReloj() y en setup() no se estaba
//                teniendo en cuenta fallo en la conexión wifi después de un reset ya que se actualiza la hora con la llamada a ThingSpeak.readCreatedAt() y esta devuelve un
//                string vacio si no hay conexión con THS.
//      16-05-24: se añade esperaTiempo (100) después del cierre del bus SDI12-A, la segunda vez que se habilita para lectura de la hydros del deposito y también se añade
//                esperaTiempo(100) después del cierre del SDI12-C. Con ello se intenta evitar que con el paso del tiempo puedan caerse los diferentes buses SDI12
//v15   20-05-24: mejoras en las funciones de lectura de sondas Meter (hydros21, es2, teros12) para homogeneizar errores, ya que estas sondas pueden dar valores negativos de error
//                p.ej. -9999, -9990, -9991, etc.
//v16   20-06-24: se implementa un reset de la alimentación de los buses SDI-12 (POWER_PINB1) ante fallos permanentes de dichos buses
//v17   03-07-24: se implementa reset SW del micro por si no funciona el reset de alimentación de los buses SDI-12.
//                Descomentar las líneas de codigo necesarias (estan donde el reset alim SDI12). De momento no se hace reset SW del micro sino que se incrementa el tiempo de
//                 reset de la alimentación de SDI12 a 5 segundos en lugar de 1 seg.
//v18   19/09/24: se implementa reset de alimentación del micro ya que los buses SDI-12 siguen sin recuperarse con las acciones anteriores. Se implementa el reset a través de
//                placa diseñada a tal fin y relé nº4 de la placa de relés (se tienen que realizar los cambios en las conexiones según esquemas). El pin del micro para realizar
//                este reset de alimentación es el pin 0.
//      16/10/24: se introduce un 'else' en el 'if' que evalua si se han perdido los 3 buses SDI-12 y que provoca el reset de alimentación. Se pone a 0 'contFallosSDI12' si no 
//                hay fallo para evitar que por acumulaciones de varios fallos en tiempos no continuos se pueda producir un reset. Solo debe haber reset si los fallos son 
//                continuos en el tiempo. También se cambia el pin reset del 4 al 27 y se pone primero como INPUTPULLDOWN por si se tiene que descargar condensador de placa reset.
//                con el pin 4 al hacer un reset en el magnetotermico de la fuente, el micro no arranca bien ¿????, pero al cambiar al 34 sí arranca bien. Puede que el pin 4
//                se haya averiado ¿¿??
//v19   19/11/24: se corrige la cabecera del fichero SD ya que no se actualizó cuando se incluyeron las nuevas sondas de Arqueta4 y Emisario2 y los ficheros SD anteriores a esta modificacion tienen
//                un descuadre entre la info y la cabecera de los ficheros


//Libreria Wifi
#include <WiFi.h>
#include <WiFiManager.h>
#include <WebServer.h>
#include <ThingSpeak.h>

//Libreria temporizador
#include <Ticker.h>

//Para control del reloj RTC
#include <RTClib.h>

//Libreria SPI para comunicaciones con memoria SD
#include <SPI.h>

//Libreria gestion ficheros
#include <FS.h>
#include <FSImpl.h>
#include <vfs_api.h>

//Libreria gestion memoria SD
#include <SD.h>
//#include <sd_defines.h>
//#include <sd_diskio.h>


//Libreria gestion bus SDI-12 de los sensores
#include <SDI12.h>

//Librerias manejo ADS1115
#include <Wire.h>
#include <Adafruit_ADS1X15.h>

#define OLE_NODO1_MICRO1

#define SERIAL_BAUD 9600 /*!< The baud rate for the output serial port */

//Pines Bus SDI-12_1
#define DATA_PINB1 23 /*!< The pin of the SDI-12 data bus */
#define POWER_PINB1 4 /*!< The sensor power pin (or -1 if not switching power) */

//Pines Bus SDI-12_2
#define DATA_PINB2 19 /*!< The pin of the SDI-12 data bus */
//#define POWER_PINB2 5 /*!< The sensor power pin (or -1 if not switching power) */

//Pines Bus SDI-12_3
#define DATA_PINB3 5

// Pines para comunicaciones SPI
#define SD_CS 13
#define SD_SCK 14
#define SD_MOSI 15
#define SD_MISO 2

// Led indicacion conexión wifi
#define PINLEDWIFI 18

//Pin caudalímetro  ultrasonidos (4-20 mA)
#define CANAL_ADC_CAUDAL 0
#define CAUDALMAX 300
const float RBUCLE = 166.4F;
const float OFFSET = 0.05;

//Pin electrovalvulas
#define ACTUACION_EV1 25
#define ACTUACION_EV2 26


#define TMAX_CONEXWIFI 180        //número máximo de segundos para intentar conexión wifi en el arranque del micro
#define TMAX_CONEXWIFI_RECONEX 5  //número máximo de segundos para intentar reconexión wifi durante ejecución del programa. Poner: 30 seg

#define NMUESTRAS 10  //número de muestras de conductividad para determinar cuando captar agua de riego

#define TMANIOBRA_EV 18  // segundos que tardan las EV de bola en realizar la maniobra apertura-cierre y viceversa

#define PINRESET 27  //pin para el reset de alimentación del micro

//Para funcion esperaTiempo
const unsigned long MAXTIMER = 4294967295;  // máximo valor de la funcion millis().

//Para comunicacion con ThingSpeak
WiFiClient cliente;                            //Instancia de la clase WiFiClient
unsigned long numCanal = 2229411;              //Numero de canal de ThingSpeak
const char *apiKey = "9VUS9PGGXKHEI3UM";       //API Key para escritura del canal de ThingSpeak
const char *apiKeyLect = "8T151IJ2QPZO48LH";   //API Key para lectura del canal de ThingSpeak
unsigned long numCanal2 = 2229416;             //Numero de canal de ThingSpeak
const char *apiKey2 = "CS6ZH1AR0APVG6XG";      //API Key para escritura del canal de ThingSpeak
const char *apiKeyLect2 = "HG5MRX4GQ2NCYZQP";  //API Key para lectura del canal de ThingSpeak

unsigned long numCanal3 = 2283243;             //Numero de canal de ThingSpeak para la ES2 del Emisario. Este es el canal del Nodo3 de Oleand
const char *apiKey3 = "Z3ZMA921O3PG9DK8";      //API Key para escritura del canal de ThingSpeak para la ES2 del Emisario. Este es el canal del Nodo3 de Oleand
const char *apiKeyLect3 = "6K7YHRND5RVG7ND2";  //API Key para lectura del canal de ThingSpeak para la ES2 del Emisario. Este es el canal del Nodo3 de Oleand

unsigned long numCanalComand = 2339720;             //Numero de canal de ThingSpeak
const char *apiKeyComand = "4L2J2BP4BIJJJODO";      //API Key para escritura del canal de ThingSpeak
const char *apiKeyLectComand = "3NO8R94L4B5XA1SH";  //API Key para escritura del canal de ThingSpeak

unsigned long numCanalParam = 2347406;             //Numero de canal de ThingSpeak
const char *apiKeyParam = "DFLY37N0UY6MM12B";      //API Key para escritura del canal de ThingSpeak
const char *apiKeyLectParam = "OUX17S01PMTO617Z";  //API Key para escritura del canal de ThingSpeak

unsigned long numCanalParam2 = 2348779;             //Numero de canal de ThingSpeak
const char *apiKeyParam2 = "E18MGZRWWA9I0XQ5";      //API Key para escritura del canal de ThingSpeak
const char *apiKeyLectParam2 = "IJPJWDOOK19YUDFO";  //API Key para escritura del canal de ThingSpeak

unsigned long numCanal4 = 2493077;             //Numero de canal de ThingSpeak para las nuevas sondas HYDROS-21 Arqueta-4 y ES2 Emisario-2 (lado norte)
const char *apiKey4 = "A037C8VELCTN9M0P";      //API Key para escritura del canal de ThingSpeak para las nuevas sondas de arqueta-4 y emisario-2
const char *apiKeyLect4 = "QSRXFNOL979B6TUK";  //API Key para lectura del canal de ThingSpeak para las nuevas sondas de arqueta-4 y emisario-2


Ticker tempThingS;                 //temporizador para enviar info a Thingspeak
int tMuestraThing = 60;            //segundos para envio de info a Thingspeak
boolean centinelaMuestra = false;  //centinela para la función del temporizador de envio de info a Thingspeak
boolean primeraEjecProg = true;

//CAMBIAR ESTAS CONSTANTES SI CAMBIA EL CAUDALMAX
const float pteF = 18.75F;  // = CAUDALMAX/16
const float cteF = 75.0F;   // = CAUDALMAX/4

Adafruit_ADS1115 ads;
float factorEscalaADC = 0.125F;

//  RTC DS1302 rtc(ce_pin, sck_pin, io_pin);
RTC_DS1307 rtc;

// Variable fichero de datos
File fichDatos;

//Direcciones SDI12 de las sondas del bus
const int HydrosDeposito = 3;  //HYDROS 21
const int ES2Tuberia = 1;      //ES2
const int HydrosArqueta1 = 4;  //HYDROS 21
const int HydrosArqueta2 = 2;  //HYDROS 21
const int ES2Emisario = 5;     //ES2

const int HydrosArqueta4 = 7;  //HYDROS 21
const int ES2Emisario2 = 6;    //ES2

//HYDROS-21
float profHydrosDeposito = 0.0;
float tempHydrosDeposito = 0.0;
float condHydrosDeposito_dS_m = 0.0;
const float MAXNIVELDEPOSITO = 2200.0;  //Maxima altura de agua en el deposito (en mm)
const float HIST_NIVELDEP = 20.0;       // Histeresis en la altura de agua en el desposito para cambio de válvulas cuando hay captación de agua

//Control deposito
Ticker tempDeposito;
boolean centinelaDeposito = false;
int tMuestraDeposito = 60;  //segundos para evaluacion de estado del deposito
float profActualDeposito = 0.0;
float profAnteriorDeposito = 0.0;
const float MIN_VAR_NIVELDEP = 20.0;  //La minima variacion en el nivel del deposito para considerar que se está llenando o vaciando (en mm)
const float RAD_DEP = 14.5;           // radio del deposito en dm
float volAcumDep = 0.0;               // volumen de liquido que se va añadiendo al deposito a lo largo del tiempo
float volExtrDep = 0.0;               // volumen de liquido que se va extrayendo del deposito a lo largo del tiempo

//HYDROS-21
float profHydrosArqueta1 = 0.0;
float tempHydrosArqueta1 = 0.0;
float condHydrosArqueta1_dS_m = 0.0;

//HYDROS-21
float profHydrosArqueta2 = 0.0;
float tempHydrosArqueta2 = 0.0;
float condHydrosArqueta2_dS_m = 0.0;

//HYDROS-21
float profHydrosArqueta4 = 0.0;
float tempHydrosArqueta4 = 0.0;
float condHydrosArqueta4_dS_m = 0.0;

//ES2 Tubería
float tempES2 = 0.0;
float condES2_dS_m = 0.0;
float MAXCONDES2 = 0.9;                                                                                    //Maxima conductividad permitida para la captura de agua. 02/02/24 Se deja en
                                                                                                           //0.9 para asegurar que no se capta agua hasta analizar datos de la ES2 para
                                                                                                           //redefinicion del calculo de medias o aplicacion de filtro SW
float MINCONDES2 = 1.0;                                                                                    //Conductividad mínima que determina que la sonda no está en contacto con líquido
float HIST_CONDES2 = 2.0;                                                                                  //Histeresis en la medida de la conductividad para cambio de válvulas cuando hay captación de agua
float muestrasCond[NMUESTRAS] = { 99.99, 99.99, 99.99, 99.99, 99.99, 99.99, 99.99, 99.99, 99.99, 99.99 };  //array de muestras de conductividad para evaluacion de captacion de agua de riego
int indMuestra = 0;                                                                                        //indice para gestion del array muestrasCond
boolean estadoCaptacion = false;                                                                           // indica si se está captando agua o no para gestionar la histeresis
int resultCond;                                                                                            //recoge el resultado de evaluar si el analisis de conductividad permite o no la captación de agua de riego
unsigned long tmuestraCond;
unsigned long tmuestraC_anterior;
float condMedia;


//ES2 Emisario
float tempES2Emisario = 0.0;
float condES2Emisario_dS_m = 0.0;

//ES2 Emisario 2 (lado norte)
float tempES2Emisario2 = 0.0;
float condES2Emisario2_dS_m = 0.0;

//Otras variables para cálculo de caudal y volumen de agua
float volumenCaudal = 0;     //Volumen agua
unsigned long dtCaudal = 0;  //variación de tiempo por cada bucle
unsigned long t0Caudal = 0;  //millis() del bucle anterior
float caudal_L_m = 0;
float volAcumCaud = 0;         //Volumen acumulado historico
float CAUDALMIN_ACUMUL = 0.5;  // l/m

Ticker tempSD;                //temporizador para grabar info en memoria SD
int tMuestraSD = 30;          //segundos para grabar info en memoria SD
boolean centinelaSD = false;  //centinela para la función del temporizador de grabar info en memoria SD

//Para lectura de canal de telecomandos y parametros de Thingspeak
String ultimaActual = "";  //String que contiene la fecha y hora del último comando en el canal Thingspeak
String nuevaActual = "";   //String que contiene la fecha y hora del último comando en el canal Thingspeak y sirve para ir leyendo dicha fecha
int comandoCaptura = 0;    //Variable para telecomando de la captura de agua: 0-captura automática, 1-Sí captura, 2-No captura

/* //Define the SDI-12 bus 
SDI12 SDI12_A(DATA_PINB1);
SDI12 SDI12_B(DATA_PINB2);
SDI12 SDI12_C(DATA_PINB3);*/

//Contador de fallos para reseteo del power de los SDI12
int MAXNUMFALLOS_SDI12 = 3;
int contFallosSDI12 = 0;


//Instancia para conexion wifi y thingspeak
WiFiManager wifiManager;

//funcion reset alimentacion del micro
void activaReset() {

  digitalWrite(PINRESET, LOW);
}

//Funcion del temporizador tikcer para Thingspeak
void callbackTempThing() {
  centinelaMuestra = true;
}

//Funcion del temporizador ticker para control del nivel del deposito y calculos de llenado y vaciado
void callbackTempDeposito() {
  centinelaDeposito = true;
}

//Funcion del temporizador tikcer para memoria SD
void callbackTempSD() {
  centinelaSD = true;
}


void sincronizaReloj() {
  String timeThingSpeak = ThingSpeak.readCreatedAt(numCanal, apiKeyLect);

  if (timeThingSpeak != "") {

#ifdef OLE_NODO1_MICRO1
    Serial.print("Time de ThingSpeak: ");
    Serial.println(timeThingSpeak);

    DateTime now = rtc.now();
    char buff[20];

    Serial.print("Time de RTC: ");
    Serial.println(now.toString(buff));
#endif

    int anio = timeThingSpeak.substring(0, 4).toInt();
    int mes = timeThingSpeak.substring(5, 7).toInt();
    int dia = timeThingSpeak.substring(8, 10).toInt();
    int hora = timeThingSpeak.substring(11, 13).toInt();
    int minuto = timeThingSpeak.substring(14, 16).toInt();
    int segundo = timeThingSpeak.substring(17, 19).toInt();

    rtc.adjust(DateTime(anio, mes, dia, hora, minuto, segundo));

#ifdef OLE_NODO1_MICRO1
    DateTime actualizada = rtc.now();
    Serial.print("Time de RTC: ");
    Serial.println(actualizada.toString(buff));
#endif
  }
}


//Lectura HYDROS-21 en bus SDI-12 para más de 1 bus SDI-12 e identificador de sonda "numSonda"
int lecturasHYDROS21(int numSonda, SDI12 act_SDI12, float *profHYDROS, float *tempHYDROS, float *condHYDROS_dSm) {
  int nSensor;
  long condHYDROS;

  String comando = String(numSonda) + "R0!";
  act_SDI12.sendCommand(comando);  //Solicitud lectura sonda
  esperaTiempo(300);               // wait a while for a response
  nSensor = act_SDI12.parseInt();

  if (nSensor != numSonda) {
#ifdef OLE_NODO1_MICRO1
    Serial.print("nSensor: ");
    Serial.println(nSensor);
    Serial.print("Problemas con el identificador de la sonda: ");
    Serial.print(numSonda);
#endif
    return (-1);
  }

  *profHYDROS = act_SDI12.parseFloat();
  *tempHYDROS = act_SDI12.parseFloat();
  condHYDROS = act_SDI12.parseInt();

  if ((*profHYDROS < -9000) || (*tempHYDROS < -9000) || (condHYDROS < -9000)) {
#ifdef OLE_NODO1_MICRO1
    Serial.print("Error en la medida devuelto por la sonda: ");
    Serial.println(nSensor);
#endif
    return (-1);
  }

  *condHYDROS_dSm = (float)condHYDROS / 1000;

#ifdef OLE_NODO1_MICRO1
  Serial.print("Sonda HYDROS-21. Nº: ");
  Serial.print(numSonda);

  Serial.print("Valor profundidad HYDROS-21 (mm): ");
  Serial.println(*profHYDROS);
  Serial.print("Temperaura HYDROS-21 (*C): ");
  Serial.println(*tempHYDROS);
  Serial.print("Conductividad HYDROS-21 (dS/m): ");
  Serial.println(*condHYDROS_dSm, 6);
  Serial.print("Conductividad HYDROS-21 (uS/cm): ");
  Serial.println(condHYDROS);
  Serial.println();
#endif

  return (1);
}

void errHydros(float *profHYDROS, float *tempHYDROS, float *condHYDROS_dSm) {

  *profHYDROS = 9999.99;
  *tempHYDROS = 9999.99;
  *condHYDROS_dSm = 9999.99;

  return;
}

//Lectura ES2 en bus SDI-12 para más de 1 bus SDI-12 e identificador de sonda "numSonda"
int lecturasES2(int numSonda, SDI12 act_SDI12, float *tempES, float *condES_dS_m) {
  int nSensor = 9999;
  long condES;

  String comando = String(numSonda) + "R0!";
  act_SDI12.sendCommand(comando);  //Solicitud lectura sonda
  esperaTiempo(300);               // wait a while for a response
  nSensor = act_SDI12.parseInt();

  if (nSensor != numSonda) {

    return (-1);
  }

  condES = act_SDI12.parseInt();
  *tempES = act_SDI12.parseFloat();

  if ((*tempES < -9000) || (condES < -9000)) {
#ifdef OLE_NODO1_MICRO1
    Serial.print("Error en la medida devuelto por la sonda: ");
    Serial.println(nSensor);
#endif
    return (-1);
  }

  *condES_dS_m = (float)condES / 1000;

  return (1);
}

void errES2(float *tempES, float *condES_dS_m) {

  *tempES = 9999.99;
  *condES_dS_m = 9999.99;

  return;
}

void lectura_caudalimetro() {

  short valorADC = ads.readADC_SingleEnded(CANAL_ADC_CAUDAL);
  float voltADC = valorADC * factorEscalaADC;

  //Función matemática que calcula la pendiente para bucle 4-20 mA según Rbucle para determinar caudal según Caudal Max. CAUDALIMETRO1 4-20mA

  caudal_L_m = (pteF * voltADC / RBUCLE) - cteF;
  float offset = (voltADC / RBUCLE) / 38;  //en calculos pte. de 1/40, ajusto a 1/38
  caudal_L_m = caudal_L_m + offset;

  if (voltADC < 80) {  //Equivale a una salida de 10 mV
#ifdef OLE_NODO1_MICRO1
    Serial.print("Caudalímetro 1 desconectado o fuera de rango\t");
#endif
    caudal_L_m = 0.0;
    return;
  }
  dtCaudal = millis() - t0Caudal;  //calculamos la variación de tiempo
  t0Caudal = millis();
  if (caudal_L_m <= OFFSET) {
    caudal_L_m = 0.0;
  }

#ifdef OLE_NODO1_MICRO1
  Serial.println("Lectura caudalimetro 1 ");
  Serial.print("voltADC: ");
  Serial.println(voltADC);
  Serial.print("Valor ADC: ");
  Serial.println(valorADC);
  Serial.print("Valor CAUDAL (l/min): ");
  Serial.println(caudal_L_m);
#endif

  float incrVol = (caudal_L_m / 60) * ((float)dtCaudal / 1000);  // volumen(L)=caudal(L/s)*tiempo(s)
  volumenCaudal = volumenCaudal + incrVol;

#ifdef OLE_NODO1_MICRO1
  Serial.print("Caudal 1: ");
  Serial.print(caudal_L_m);
  Serial.print("L/min\tVolumen 1: ");
  Serial.print(volumenCaudal, 3);
  Serial.println(" L");
  Serial.print("Incremento caudal 1:");
  Serial.println(incrVol, 3);
  Serial.print("dtCaudal 1: ");
  Serial.print(dtCaudal);
  Serial.println("  ms");
  Serial.println("------------------------------------------------");
#endif

  if ((caudal_L_m == 0.0) && (volumenCaudal > 0)) {

    volAcumCaud = volAcumCaud + volumenCaudal;
    grabarLecturasSD();
    volumenCaudal = 0;
  }

#ifdef OLE_NODO1_MICRO1
  Serial.print("Volumen acumulado 1: ");
  Serial.println(volAcumCaud);
  Serial.println();
#endif

  return;
}

//Evalua las muestras de conductividad para determinar si hay que realizar o no captación de agua de riego
int evaluaCond(float cond_dS_m, int ind, boolean estadoCapt) {

  float sumaCond = 0.0;

  muestrasCond[ind] = cond_dS_m;
  for (int i = 0; i < NMUESTRAS; i++) {
    sumaCond = sumaCond + muestrasCond[i];
  }

  condMedia = sumaCond / NMUESTRAS;

#ifdef OLE_NODO1_MICRO1
  Serial.print("Conductividad media en tuberia (dep. intermedio): ");
  Serial.println(condMedia);
#endif

  if (estadoCapt == false) {
    if (condMedia <= MAXCONDES2) return (1);
    else return (-1);
  } else {
    if (condMedia <= (MAXCONDES2 + HIST_CONDES2)) return (1);
    else return (-1);
  }
}

void grabarLecturasSD() {
  //Obtencion de fecha y hora
  DateTime now = rtc.now();

  fichDatos = SD.open("/Nodo1Micro1_Oleand.txt", FILE_APPEND);
  if (fichDatos) {
    fichDatos.print(now.day());
    fichDatos.print("/");
    fichDatos.print(now.month());
    fichDatos.print("/");
    fichDatos.print(now.year());
    fichDatos.print(" ");
    fichDatos.print(now.hour());
    fichDatos.print(":");
    fichDatos.print(now.minute());
    fichDatos.print(":");
    fichDatos.print(now.second());
    fichDatos.print(",");
    fichDatos.print(profHydrosDeposito);
    fichDatos.print(",");
    fichDatos.print(tempHydrosDeposito);
    fichDatos.print(",");
    fichDatos.print(condHydrosDeposito_dS_m);
    fichDatos.print(",");
    fichDatos.print(profHydrosArqueta1);
    fichDatos.print(",");
    fichDatos.print(tempHydrosArqueta1);
    fichDatos.print(",");
    fichDatos.print(condHydrosArqueta1_dS_m);
    fichDatos.print(",");
    fichDatos.print(profHydrosArqueta2);
    fichDatos.print(",");
    fichDatos.print(tempHydrosArqueta2);
    fichDatos.print(",");
    fichDatos.print(condHydrosArqueta2_dS_m);
    fichDatos.print(",");
    fichDatos.print(tempES2);
    fichDatos.print(",");
    fichDatos.print(condES2_dS_m);
    fichDatos.print(",");
    fichDatos.print(condMedia);
    fichDatos.print(",");
    fichDatos.print(volumenCaudal);
    fichDatos.print(",");
    fichDatos.print(caudal_L_m);
    fichDatos.print(",");
    fichDatos.print(volAcumCaud);
    fichDatos.print(",");
    fichDatos.print(volAcumDep);
    fichDatos.print(",");
    fichDatos.print(volExtrDep);
    fichDatos.print(",");
    fichDatos.print(tempES2Emisario);
    fichDatos.print(",");
    fichDatos.print(condES2Emisario_dS_m);
    fichDatos.print(",");
    fichDatos.print(profHydrosArqueta4);
    fichDatos.print(",");
    fichDatos.print(tempHydrosArqueta4);
    fichDatos.print(",");
    fichDatos.print(condHydrosArqueta4_dS_m);
    fichDatos.print(",");
    fichDatos.print(tempES2Emisario2);
    fichDatos.print(",");
    fichDatos.println(condES2Emisario2_dS_m);

  } else {
#ifdef OLE_NODO1_MICRO1
    Serial.println("Error al abrir el fichero de datos 'Nodo1Micro1_Oleand.txt'");
#endif
  }
  fichDatos.close();
}

void abrirEV(byte nEV) {  //Se cambian las logicas por pruebas a través de leds

  // nEV 1 = EV NA (para abrir HIGH, para cerrar LOW)
  // nEV 2 = EV NC (para abrir LOW, para cerrar HIGH)
#ifdef OLE_NODO1_MICRO1
  Serial.print("Abierta EV nº: ");
  Serial.println(nEV);
#endif

  if (nEV == 1) {
    digitalWrite(ACTUACION_EV1, HIGH);  //solenoide no excitado
  }
  if (nEV == 2) {
    digitalWrite(ACTUACION_EV2, LOW);  //solenoide excitada
  }
  return;
}

void cerrarEV(byte nEV) {  //Se cambian las logicas por pruebas a través de leds


  // nEV 1 = EV NA (para abrir HIGH, para cerrar LOW)
  // nEV 2 = EV NC (para abrir LOW, para cerrar HIGH)
#ifdef OLE_NODO1_MICRO1
  Serial.print("Cerrada EV nº: ");
  Serial.println(nEV);
#endif

  if (nEV == 1) {
    digitalWrite(ACTUACION_EV1, LOW);  //solenoide excitado
  }
  if (nEV == 2) {
    digitalWrite(ACTUACION_EV2, HIGH);  //solenoide no excitado
  }
  return;
}

int comparaFechas(String primeraFecha, String segundaFecha) {

  //devuelve 1 si primeraFecha>segundaFecha, 0 si primeraFecha=segundaFecha, 2 si primeraFecha<segundaFecha

  int anio1 = primeraFecha.substring(0, 4).toInt();
  int mes1 = primeraFecha.substring(5, 7).toInt();
  int dia1 = primeraFecha.substring(8, 10).toInt();
  int hora1 = primeraFecha.substring(11, 13).toInt();
  int minuto1 = primeraFecha.substring(14, 16).toInt();
  int segundo1 = primeraFecha.substring(17, 19).toInt();

  int anio2 = segundaFecha.substring(0, 4).toInt();
  int mes2 = segundaFecha.substring(5, 7).toInt();
  int dia2 = segundaFecha.substring(8, 10).toInt();
  int hora2 = segundaFecha.substring(11, 13).toInt();
  int minuto2 = segundaFecha.substring(14, 16).toInt();
  int segundo2 = segundaFecha.substring(17, 19).toInt();

  if (anio1 > anio2) return (1);
  else if (anio1 < anio2) return (2);
  else {  //mismo anio
    if (mes1 > mes2) return (1);
    else if (mes1 < mes2) return (2);
    else {  //mismo mes
      if (dia1 > dia2) return (1);
      else if (dia1 < dia2) return (2);
      else {  //mismo dia
        if (hora1 > hora2) return (1);
        else if (hora1 < hora2) return (2);
        else {  //misma hora
          if (minuto1 > minuto2) return (1);
          else if (minuto1 < minuto2) return (2);
          else {  //mismo minuto
            if (segundo1 > segundo2) return (1);
            else if (segundo1 < segundo2) return (2);
            else return (0);
          }
        }
      }
    }
  }
}


void setup() {

  //Inicio monitor serie
  Serial.begin(SERIAL_BAUD);

  //Configuración pin reset alimentacion micro
  pinMode(PINRESET, INPUT_PULLDOWN);
  digitalRead(PINRESET);
  delay(1000);
  pinMode(PINRESET, OUTPUT);
  delay(100);
  digitalWrite(PINRESET, HIGH);



  // Configuracion pines electrovalvulas

  pinMode(ACTUACION_EV1, OUTPUT);
  pinMode(ACTUACION_EV2, OUTPUT);

  digitalWrite(ACTUACION_EV1, HIGH);  //EV1 queda abierta con HIGH ya que se conecta el NC del relé al borne 3 de la EV
  digitalWrite(ACTUACION_EV2, HIGH);  //EV2 queda cerrada con HIGH ya que se conecta el NC del relé al borne 2 de la EV
  // Se define y activa led de conexión wifi que indica que el equipo no está conectado a wifi
  pinMode(PINLEDWIFI, OUTPUT);
  delay(10);
  digitalWrite((PINLEDWIFI), HIGH);

  //Inicio comunicaciones SPI con declaración de los pines utilizados

  SPI.begin(SD_SCK, SD_MISO, SD_MOSI);  // SPI.begin(SCK, MISO, MOSI, SS);

  delay(500);  //Estabilización I2C?

  // Inicio reloj RTC
  rtc.begin();
  delay(500);
  if (!rtc.isrunning()) {
#ifdef OLE_NODO1_MICRO1
    Serial.println("RTC is NOT running!");
#endif
    rtc.adjust(DateTime(__DATE__, __TIME__));
    while (1)
      ;
  }

  // following line sets the RTC to the date & time this sketch was compiled
  /*rtc.adjust(DateTime(__DATE__, __TIME__));
  Serial.println(__DATE__);
  Serial.println(__TIME__);
  Serial.println("Configurada hora carga programa");*/

  //Inicio convertidor ADC ADS1115
  ads.setGain(GAIN_ONE);
  ads.begin();

  //Inicio tarjeta SD
  if (!SD.begin(SD_CS)) {
#ifdef OLE_NODO1_MICRO1
    Serial.println("Card Mount Failed");
#endif
    return;
  }

  if (!SD.exists("/Nodo1Micro1_Oleand.txt")) {
    fichDatos = SD.open("/Nodo1Micro1_Oleand.txt", FILE_WRITE);
    if (fichDatos) {
#ifdef OLE_NODO1_MICRO1
      Serial.println("Archivo nuevo, Escribiendo encabezado(fila 1)");
#endif
      fichDatos.print("Fecha,ProfDeposito_OleN1 (mm),TempDeposito_OleN1 (C),CondDeposito_OleN1 (dS/m),ProfArqueta1_OleN1 (mm),TempArqueta1_OleN1 (C),CondArqueta1_OleN1 (dS/m),");
      fichDatos.print("ProfArqueta2_OleN1 (mm),TempArqueta2_OleN1 (C),CondArqueta2_OleN1 (dS/m),TempES2_OleN1 (C),CondES2_Instant_OleN1 (dS/m),CondES2_Media_OleN1 (dS/m),");
      fichDatos.print("Volumen Caudal (l),Caudal (l/min),Acumulado Caudal (l),Volumen Acumulado Deposito (l),Volumen Extraido Deposito (l),TempES2Emisario (C), ConductES2Emisario (dS/m),");
      fichDatos.println("ProfArqueta4_OleN1 (mm),TempArqueta4_OleN1 (C),CondArqueta4_OleN1 (dS/m),TempES2Emisario2 (C),ConductES2Emisario2 (dS/m)");
      fichDatos.close();
    } else {
#ifdef OLE_NODO1_MICRO1
      Serial.println("Error creando el archivo Nodo1Micro1_Oleand");
#endif
      while (1)
        ;
    }
  }

  //energizacion power pin SDI-12

  pinMode(POWER_PINB1, OUTPUT);
  digitalWrite(POWER_PINB1, HIGH);
  esperaTiempo(200);


  pinMode(DATA_PINB1, INPUT);
  pinMode(DATA_PINB2, INPUT);
  pinMode(DATA_PINB3, INPUT);


  /* SDI12_B.begin();
  esperaTiempo(500);  // allow things to settle
  pinMode(POWER_PINB2, OUTPUT);
  digitalWrite(POWER_PINB2, HIGH);
  esperaTiempo(200);*/


#ifdef OLE_NODO1_MICRO1
  Serial.println("MEDIDAS SOBRE EL SDI-12 A y B");
#endif

  //Conexion wifi y thingspeak

  wifiManager.setConfigPortalTimeout(TMAX_CONEXWIFI);
  boolean exitoConexion = wifiManager.autoConnect("Nodo1OLE");
  if (!exitoConexion) {
#ifdef OLE_NODO1_MICRO1
    Serial.println("[WIFI] Error en la conexión WiFi. Timeout");
#endif
    // ESP.restart();
    // delay(1000);
  } else {
#ifdef OLE_NODO1_MICRO1
    Serial.println("[WIFI] Conectado a la red WiFi");
#endif
    digitalWrite(PINLEDWIFI, LOW);
  }

  ThingSpeak.begin(cliente);

  //activación del temporizador para envio de info a Thingspeak
  tempThingS.attach(tMuestraThing, callbackTempThing);

  //activación del temporizador para evaluacion de estado de deposito
  tempDeposito.attach(tMuestraDeposito, callbackTempDeposito);

  //activación del temporizador para grabar info en SD
  tempSD.attach(tMuestraSD, callbackTempSD);

  tmuestraCond = millis();  //inicio tiempo para la muestra de conductividad

  ultimaActual = ThingSpeak.readCreatedAt(numCanalComand, apiKeyLectComand);  //Lectura de fecha y hora de la última actualización del canal de telecomando
  comandoCaptura = 0;                                                         //Se inicializa el telecomando de captura de agua en modo automático (según lógica del programa)

  fichDatos.close();  // para asegurar cierre de fichero en caso de reset
}

void loop() {

  int respuesta;

#ifdef OLE_NODO1_MICRO1
  DateTime now = rtc.now();
  Serial.print("Fecha y hora: ");
  Serial.print(now.day());
  Serial.print("/");
  Serial.print(now.month());
  Serial.print("/");
  Serial.print(now.year());
  Serial.print(" ");
  Serial.print(now.hour());
  Serial.print(":");
  Serial.print(now.minute());
  Serial.print(":");
  Serial.println(now.second());
#endif

  SDI12 SDI12_A(DATA_PINB1);
  SDI12_A.begin();

  esperaTiempo(100);  //El 25-04-24 se añade esta linea pq con nuevo micro cambiado no funciona el doble bus SDI-12 tal y como estaba el SW

  if (lecturasHYDROS21(HydrosArqueta1, SDI12_A, &profHydrosArqueta1, &tempHydrosArqueta1, &condHydrosArqueta1_dS_m) < 1) {
    errHydros(&profHydrosArqueta1, &tempHydrosArqueta1, &condHydrosArqueta1_dS_m);
  }

  if (lecturasHYDROS21(HydrosDeposito, SDI12_A, &profHydrosDeposito, &tempHydrosDeposito, &condHydrosDeposito_dS_m) < 1) {
    errHydros(&profHydrosDeposito, &tempHydrosDeposito, &condHydrosDeposito_dS_m);
  }

  if (lecturasHYDROS21(HydrosArqueta2, SDI12_A, &profHydrosArqueta2, &tempHydrosArqueta2, &condHydrosArqueta2_dS_m) < 1) {
    errHydros(&profHydrosArqueta2, &tempHydrosArqueta2, &condHydrosArqueta2_dS_m);
  }


  SDI12_A.end();
  esperaTiempo(100);  //El 25-04-24 se añade esta linea pq con nuevo micro cambiado no funciona el doble bus SDI-12 tal y como estaba el SW

  SDI12 SDI12_B(DATA_PINB2);
  SDI12_B.begin();

  esperaTiempo(100);  //El 25-04-24 se añade esta linea pq con nuevo micro cambiado no funciona el doble bus SDI-12 tal y como estaba el SW

  if (lecturasES2(ES2Tuberia, SDI12_B, &tempES2, &condES2_dS_m) < 1) {
    errES2(&tempES2, &condES2_dS_m);
    Serial.println("Error lectura ES2 del deposito intermedio/tuberia");  //quitar linea tras depurar
  }

  if (lecturasES2(ES2Emisario, SDI12_B, &tempES2Emisario, &condES2Emisario_dS_m) < 1) {
#ifdef OLE_NODO1_MICRO1
    Serial.print("error en lectura ES2: ");
    Serial.println(ES2Emisario);
#endif
    errES2(&tempES2Emisario, &condES2Emisario_dS_m);
  }

  SDI12_B.end();
  esperaTiempo(100);  //El 25-04-24 se añade esta linea pq con nuevo micro cambiado no funciona el doble bus SDI-12 tal y como estaba el SW

  SDI12 SDI12_C(DATA_PINB3);
  SDI12_C.begin();

  esperaTiempo(100);  //El 25-04-24 se añade esta linea pq con nuevo micro cambiado no funciona el doble bus SDI-12 tal y como estaba el SW

  if (lecturasHYDROS21(HydrosArqueta4, SDI12_C, &profHydrosArqueta4, &tempHydrosArqueta4, &condHydrosArqueta4_dS_m) < 1) {
    errHydros(&profHydrosArqueta4, &tempHydrosArqueta4, &condHydrosArqueta4_dS_m);
  }

  if (lecturasES2(ES2Emisario2, SDI12_C, &tempES2Emisario2, &condES2Emisario2_dS_m) < 1) {
#ifdef OLE_NODO1_MICRO1
    Serial.print("error en lectura ES2-2: ");
    Serial.println(ES2Emisario2);
#endif
    errES2(&tempES2Emisario2, &condES2Emisario2_dS_m);
  }

  SDI12_C.end();
  esperaTiempo(100);  //El 16-05-24 se añade esta linea



  lectura_caudalimetro();

#ifdef OLE_NODO1_MICRO1
  Serial.print("caudal: ");
  Serial.println(caudal_L_m);
  Serial.print("conductividad ES2: ");
  Serial.println(condES2_dS_m, 4);
  Serial.print("profundidad Hydros21: ");
  Serial.println(profHydrosDeposito);
  Serial.print("conductividad ES2 emisario: ");
  Serial.println(condES2Emisario_dS_m, 4);
#endif

  // Se evaluan las muestras de conductividad de tubería (deposito intermedio) para determinar si hay que realizar captación de agua o no
  if ((condES2_dS_m > MINCONDES2) && (condES2_dS_m < 9999.9)) {  //si la lectura de la ES2 de tubería es menor de un valor mínimo, no se tiene en cuenta esa lectura
    resultCond = evaluaCond(condES2_dS_m, indMuestra, estadoCaptacion);
    indMuestra = ++indMuestra;
    if (indMuestra >= NMUESTRAS) {
      indMuestra = 0;
    }
  }

  tmuestraC_anterior = tmuestraCond;
  tmuestraCond = millis();

#ifdef OLE_NODO1_MICRO1
  Serial.print("Tiempo entre muestras de conductividad (mS): ");
  Serial.println(tmuestraCond - tmuestraC_anterior);
  Serial.print("Valor de la muestra: ");
  Serial.println(condES2_dS_m);
  Serial.println("-------------------");
  Serial.print("Conductividad media: ");
  Serial.println(condMedia);
  Serial.print("Resultado conductividad media para captación (1 captar; -1 no captar): ");
  Serial.println(resultCond);
#endif

  if (((resultCond > 0) && (profHydrosDeposito < MAXNIVELDEPOSITO) && (caudal_L_m > CAUDALMIN_ACUMUL) && (comandoCaptura < 2)) || ((profHydrosDeposito < MAXNIVELDEPOSITO) && (comandoCaptura == 1))) {
    abrirEV(2);
    esperaTiempo(5000);
    cerrarEV(1);
    estadoCaptacion = true;
  } else if ((resultCond < 0) || (profHydrosDeposito > (MAXNIVELDEPOSITO + HIST_NIVELDEP)) || (caudal_L_m <= CAUDALMIN_ACUMUL) || (comandoCaptura == 2)) {
    abrirEV(1);
    esperaTiempo(5000);
    cerrarEV(2);
    estadoCaptacion = false;
  }

  /*
  if ((caudal_L_m > 0) && (condES2_dS_m < MAXCONDES2) && (condES2_dS_m > MINCONDES2) && (profHydrosDeposito < MAXNIVELDEPOSITO)) {  //Se podría quitar la condición (condES2_dS_m > MINCONDES2) si hay contacto de liquido cuando hay caudal
    abrirEV(2);
    esperaTiempo(1000);
    cerrarEV(1);
  } else if ((caudal_L_m == 0) || (condES2_dS_m > (MAXCONDES2 + HIST_CONDES2)) || (profHydrosDeposito > (MAXNIVELDEPOSITO + HIST_NIVELDEP))) {
    abrirEV(1);
    esperaTiempo(1000);
    cerrarEV(2);
    if (caudal_L_m == 0) {
      volAcumCaud = volAcumCaud + volumenCaudal;
      volumenCaudal = 0;
    }
  }*/

  if (centinelaDeposito) {

    SDI12 SDI12_A(DATA_PINB1);
    SDI12_A.begin();

    esperaTiempo(100);  //El 25-04-24 se añade esta linea pq con nuevo micro cambiado no funciona el doble bus SDI-12 tal y como estaba el SW

    if (lecturasHYDROS21(HydrosDeposito, SDI12_A, &profHydrosDeposito, &tempHydrosDeposito, &condHydrosDeposito_dS_m) > 0) {
      profAnteriorDeposito = profActualDeposito;
      profActualDeposito = profHydrosDeposito;

      if ((profActualDeposito - profAnteriorDeposito) > MIN_VAR_NIVELDEP) {

        volAcumDep = volAcumDep + ((profActualDeposito - profAnteriorDeposito) * 3.1416 * RAD_DEP * RAD_DEP / 100);  //en litros, se utilizan dm

      } else if ((profAnteriorDeposito - profActualDeposito) > MIN_VAR_NIVELDEP) {

        volExtrDep = volExtrDep + ((profActualDeposito - profAnteriorDeposito) * 3.1416 * RAD_DEP * RAD_DEP / 100);  //en litros, se utilizan dm
      }
    } else {
#ifdef OLE_NODO1_MICRO1
      Serial.print("Error para calculo de volumnen acumulado o extraido en la lectura de la Hydros del depósito con Id: ");
      Serial.println(HydrosDeposito);
#endif
    }

    SDI12_A.end();
    esperaTiempo(100);  //El 16-05-24 se añade esta linea
  }

  //Envio de datos a thingspeak y grabado en memoria SD
  if (centinelaMuestra) {
    ThingSpeak.setField(1, profHydrosDeposito);
    ThingSpeak.setField(2, condHydrosDeposito_dS_m);
    ThingSpeak.setField(3, volAcumDep);
    ThingSpeak.setField(4, volExtrDep);
    ThingSpeak.setField(5, volAcumCaud);
    ThingSpeak.setField(6, condES2_dS_m);
    ThingSpeak.setField(7, condES2Emisario_dS_m);
    ThingSpeak.setField(8, tempES2Emisario);

    respuesta = ThingSpeak.writeFields(numCanal, apiKey);
    centinelaMuestra = false;

    if (respuesta == TS_OK_SUCCESS) {
#ifdef OLE_NODO1_MICRO1
      Serial.print("[THSP] Envio de datos correcto: ");
      Serial.println(respuesta);
#endif
    } else if (respuesta == TS_ERR_CONNECT_FAILED) {

#ifdef OLE_NODO1_MICRO1
      Serial.print("getWLSStatusString: ");
      Serial.println(wifiManager.getWLStatusString());
#endif
      if (wifiManager.getWLStatusString() != "WL_CONNECTED") {
#ifdef OLE_NODO1_MICRO1
        Serial.print("[WIFI] Perdida conexion WIFI. Reconectando");
#endif

        digitalWrite(PINLEDWIFI, HIGH);
        wifiManager.setConfigPortalTimeout(TMAX_CONEXWIFI_RECONEX);
        boolean exitoConexion = wifiManager.autoConnect("Nodo1OLE");
        if (!exitoConexion) {
#ifdef OLE_NODO1_MICRO1
          Serial.println("[WIFI] Error en la conexión WiFi. Timeout");
#endif
        } else {
#ifdef OLE_NODO1_MICRO1
          Serial.println("[WIFI] Conectado a la red WiFi");
#endif
          digitalWrite(PINLEDWIFI, LOW);
        }
      } else {
        ThingSpeak.begin(cliente);
      }

    } else {
#ifdef OLE_NODO1_MICRO1
      Serial.print("[THSD] Error en el envío de datos al canal 2229411: ");
      Serial.println(respuesta);
#endif
    }
    //Sincroniza RTC con fecha y hora de ThingSpeak, la primera vez después de un reset o reinicio, siempre y cuando haya conexión con ThingSpeak;
    if ((primeraEjecProg) && (respuesta == TS_OK_SUCCESS)) {
      sincronizaReloj();
      primeraEjecProg = false;
    }

    ThingSpeak.setField(1, volumenCaudal);
    ThingSpeak.setField(2, caudal_L_m);
    ThingSpeak.setField(3, tempES2);
    ThingSpeak.setField(4, profHydrosArqueta1);
    ThingSpeak.setField(5, condHydrosArqueta1_dS_m);
    ThingSpeak.setField(6, profHydrosArqueta2);
    ThingSpeak.setField(7, condHydrosArqueta2_dS_m);
    ThingSpeak.setField(8, condMedia);

    //ThingSpeak.setField(8, 0);

    respuesta = ThingSpeak.writeFields(numCanal2, apiKey2);

    if (respuesta == TS_OK_SUCCESS) {
#ifdef OLE_NODO1_MICRO1
      Serial.print("[THSP] Envio de datos correcto: ");
      Serial.println(respuesta);
#endif
    } else {
#ifdef OLE_NODO1_MICRO1
      Serial.print("[THSD] Error en el envío de datos al canal 2229416: ");
      Serial.println(respuesta);
#endif
    }


    //PARTE NUEVO CANAL THS DONDE SUBIR DATOS DE HYDROS ARQUETA-4 Y ES2 EMISARIO-2
    ThingSpeak.setField(1, profHydrosArqueta4);
    ThingSpeak.setField(2, condHydrosArqueta4_dS_m);
    ThingSpeak.setField(3, tempHydrosArqueta4);
    ThingSpeak.setField(4, condES2Emisario2_dS_m);
    ThingSpeak.setField(5, tempES2Emisario2);

    respuesta = ThingSpeak.writeFields(numCanal4, apiKey4);

    if (respuesta == TS_OK_SUCCESS) {
#ifdef OLE_NODO1_MICRO1
      Serial.print("[THSP] Envio de datos correcto: ");
      Serial.println(respuesta);
#endif
    } else {
#ifdef OLE_NODO1_MICRO1
      Serial.print("[THSD] Error en el envío de datos al canal 2493077: ");
      Serial.println(respuesta);
#endif
    }


    //PARTE DE LECTURA DE CANAL DE TELECOMANDO Y ASIGNACION DE VARIABLES

    respuesta = ThingSpeak.readMultipleFields(numCanalComand, apiKeyLectComand);

    if (respuesta == TS_OK_SUCCESS) {
#ifdef OLE_NODO1_MICRO1
      Serial.print("[THSP] Lectura de datos telecomando correcto: ");
      Serial.println(respuesta);
#endif

      nuevaActual = ThingSpeak.getCreatedAt();
      int resCompFechas = comparaFechas(ultimaActual, nuevaActual);

      if (resCompFechas == 2) {
        ultimaActual = nuevaActual;

        comandoCaptura = ThingSpeak.getFieldAsInt(1);
        if ((comandoCaptura < 0) || (comandoCaptura > 2)) comandoCaptura = 0;

        float lect_MAXCONDES2 = ThingSpeak.getFieldAsFloat(2);
        if (lect_MAXCONDES2 > 0) MAXCONDES2 = lect_MAXCONDES2;

        float lect_MINCONDES2 = ThingSpeak.getFieldAsFloat(3);
        if (lect_MINCONDES2 > 0) MINCONDES2 = lect_MINCONDES2;

        float lect_HIST_CONDES2 = ThingSpeak.getFieldAsFloat(4);
        if (lect_HIST_CONDES2 > 0) HIST_CONDES2 = lect_HIST_CONDES2;
      }

    } else {
#ifdef OLE_NODO1_MICRO1
      Serial.print("[THSD] Error en lectura de datos del canal 2339720: ");
      Serial.println(respuesta);
      comandoCaptura = 0;  // Se deja la captación de agua en automático cuando después de intento de reconexión a Wifi sigue sin establecerse
#endif
    }

    ThingSpeak.setField(1, MAXCONDES2);
    ThingSpeak.setField(2, MINCONDES2);
    ThingSpeak.setField(3, HIST_CONDES2);

    respuesta = ThingSpeak.writeFields(numCanalParam, apiKeyParam);

    if (respuesta == TS_OK_SUCCESS) {
#ifdef OLE_NODO1_MICRO1
      Serial.print("[THSP] Envio de datos correcto: ");
      Serial.println(respuesta);
#endif
    } else {
#ifdef OLE_NODO1_MICRO1
      Serial.print("[THSD] Error en el envío de datos al canal 2347406: ");
      Serial.println(respuesta);
#endif
    }

    ThingSpeak.setField(1, comandoCaptura);
    respuesta = ThingSpeak.writeFields(numCanalParam2, apiKeyParam2);

    if (respuesta == TS_OK_SUCCESS) {
#ifdef OLE_NODO1_MICRO1
      Serial.print("[THSP] Envio de datos correcto: ");
      Serial.println(respuesta);
#endif
    } else {
#ifdef OLE_NODO1_MICRO1
      Serial.print("[THSD] Error en el envío de datos al canal 2348779: ");
      Serial.println(respuesta);
#endif
    }
  }

  if (centinelaSD) {
    grabarLecturasSD();
    centinelaSD = false;
  }

  /* //pruebas SW borrar
  condHydrosDeposito_dS_m = 9999.99;
  condES2_dS_m = 9999.99;
  condES2Emisario_dS_m = 9999.99;
  condHydrosArqueta1_dS_m = 9999.99;
  condHydrosArqueta2_dS_m = 9999.99;
  condHydrosArqueta4_dS_m = 9999.99;
  condES2Emisario2_dS_m = 9999.99;
  //fin codigo pruebas SW*/

  //chequeto de funcionamiento de los buses SDI-12 y reseteo de alimentación en caso de fallo continuo
  if ((condHydrosDeposito_dS_m >= 9999.99) && (condES2_dS_m >= 9999.99) && (condES2Emisario_dS_m >= 9999.99) && (condHydrosArqueta1_dS_m >= 9999.99)
   && (condHydrosArqueta2_dS_m >= 9999.99) && (condHydrosArqueta4_dS_m >= 9999.99) && (condES2Emisario2_dS_m >= 9999.99)) {

    contFallosSDI12++;
#ifdef OLE_NODO1_MICRO1
    Serial.println("......................");
    Serial.println("Fallo simultaneo de todos los buses SDI-12");
    Serial.print("valor contador fallos SDI-12 (si >= 3 provoca reset): ");
    Serial.println(contFallosSDI12);
    Serial.println("......................");
#endif

    if (contFallosSDI12 >= MAXNUMFALLOS_SDI12) {
      /* //reset de la alimentación de los buses SDI-12
      digitalWrite(POWER_PINB1, LOW);
      esperaTiempo(5000); //En v16 estaba a 1000 mseg. y no funciona. Incremento a 5000 mseg para chequear si funciona por si fuese necesario 
                          // más tiempo sin energizar los buses. Si tampoco funciona realizar Reset SW descomentando linea más abajo
      digitalWrite(POWER_PINB1, HIGH);
      esperaTiempo(500);

      contFallosSDI12 = 0;
      //esp_restart(); //opcion reset del micro por SW*/
      activaReset();
    }
  } else contFallosSDI12 = 0;
}

void esperaTiempo(unsigned long tiempo) {

  unsigned long tiempoEntrada = millis();

  if ((MAXTIMER - tiempoEntrada) < tiempo) {
    if (millis() < tiempoEntrada) {
      tiempo = tiempo - (MAXTIMER - tiempoEntrada);
      tiempoEntrada = 0;
    }
  }

  while ((millis() - tiempoEntrada) < tiempo) {
    if ((MAXTIMER - tiempoEntrada) < tiempo) {
      if (millis() < tiempoEntrada) {
        tiempo = tiempo - (MAXTIMER - tiempoEntrada);
        tiempoEntrada = 0;
      }
    }
  }
  return;
}