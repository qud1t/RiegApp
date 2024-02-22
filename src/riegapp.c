//==================[inlcusiones]============================================/

#include <string.h>

#include <stdlib.h>

#include "sapi.h" // <= Biblioteca sAPI

//==================[definiciones y macros]==================================/

#define UART_PC UART_USB
#define UART_BLUETOOTH UART_232
#define HUMEDAD_AIRE 650
#define HUMEDAD_AGUA 250
#define WATER_PIN GPIO3
#define RED_LED GPIO5
#define GREEN_LED GPIO7
#define BLUE_LED GPIO8
#define BAUDRATE 9600

uint8_t recipient_height = 6;
uint8_t min_humidity = 50;
uint32_t scan_period = 60000;
uint32_t elapsed_time = 0;
delay_t nb_delay;

//==================[definiciones de datos internos]=========================/

//==================[definiciones de datos externos]=========================/

void turnLeds(int on);

//==================[declaraciones de funciones internas]====================/

void sendTo(int pin, int distancia, int humedad, const char *estado)
{
  stdioPrintf(pin, "{\"water_level\": %d, \"humidity\": %d, \"water_pump\": %s, \"min_humidity\": %d, \"recipient_height\": %d, \"scan_period\": %d }", distancia, humedad, estado, min_humidity, recipient_height, scan_period);
}

float calculateHumidityPercentage(uint16_t value)
{
  float percentage = (100 - ((value / 1023.00) * 100));
  if (percentage < 0)
  {
    return 0.0;
  }
  else if (percentage > 100)
  {
    return 100.0;
  }
  else
  {
    return percentage;
  }
}

char *itoa(int value, char *result, int base)
{
  // check that the base if valid
  if (base < 2 || base > 36)
  {
    *result = '\0';
    return result;
  }

  char *ptr = result, *ptr1 = result, tmp_char;
  int tmp_value;

  do
  {
    tmp_value = value;
    value /= base;
    *ptr++ = "zyxwvutsrqponmlkjihgfedcba9876543210123456789abcdefghijklmnopqrstuvwxyz"[35 + (tmp_value - value * base)];
  } while (value);

  // Apply negative sign
  if (tmp_value < 0)
    *ptr++ = '-';
  *ptr-- = '\0';
  while (ptr1 < ptr)
  {
    tmp_char = *ptr;
    *ptr-- = *ptr1;
    *ptr1++ = tmp_char;
  }
  return result;
}

void readSoilOnTick(void *ptr)
{
  if (delayRead(&nb_delay))
  {
    gpioMap_t adc = (gpioMap_t)ptr;
    /* Leo la Entrada Analogica AI0 - ADC0 CH1 */
    uint16_t muestra = adcRead(adc);

    /* Buffer */
    static char uartBuff[10];
    static char percentage[10];
    /* Conversi�n de muestra entera a ascii con base decimal */
    uint32_t distanceInCms = ultrasonicSensorGetDistance(ULTRASONIC_SENSOR_0, CM);

    itoa(muestra, uartBuff, 10); /* 10 significa decimal */
    float porcentaje = calculateHumidityPercentage(muestra);

    sendTo(UART_BLUETOOTH, (int)distanceInCms, (int)porcentaje, "false");
    sendTo(UART_PC, (int)distanceInCms, (int)porcentaje, "false");
    uartWriteString(UART_PC, "\r\n");

    if (porcentaje < min_humidity)
    {
      if (distanceInCms < recipient_height)
      {
        while (porcentaje < min_humidity && distanceInCms < recipient_height)
        {
          // Leo de nuevo los valores para ver como cambiaron
          muestra = adcRead(adc);
          itoa(muestra, uartBuff, 10); /* 10 significa decimal */
          porcentaje = calculateHumidityPercentage(muestra);
          distanceInCms = ultrasonicSensorGetDistance(ULTRASONIC_SENSOR_0, CM);

          gpioWrite(WATER_PIN, ON);

          sendTo(UART_BLUETOOTH, (int)distanceInCms, (int)porcentaje, "true");
          sendTo(UART_PC, (int)distanceInCms, (int)porcentaje, "true");
          uartWriteString(UART_PC, "\r\n");

          turnLeds(0);
          gpioWrite(GREEN_LED, 0);
          gpioWrite(RED_LED, 0);
        }
        gpioWrite(WATER_PIN, OFF);
        if (distanceInCms > 6)
        {
          turnLeds(0);
          gpioWrite(GREEN_LED, 0);

          sendTo(UART_BLUETOOTH, (int)distanceInCms, (int)porcentaje, "false");
          sendTo(UART_PC, (int)distanceInCms, (int)porcentaje, "false");
          uartWriteString(UART_PC, "\r\n");
        }
        else
        {
          turnLeds(0);
          gpioWrite(RED_LED, 0);

          sendTo(UART_BLUETOOTH, (int)distanceInCms, (int)porcentaje, "false");
          sendTo(UART_PC, (int)distanceInCms, (int)porcentaje, "false");
          uartWriteString(UART_PC, "\r\n");
        }
      }
      else
      {
        turnLeds(0);
        gpioWrite(RED_LED, 0);
      }
    }
    else
    {
      turnLeds(0);
      gpioWrite(GREEN_LED, 0);
    }
  }
}

//==================[declaraciones de funciones externas]====================/

bool_t readCommandAndValue(uint8_t *command, uint8_t *value, int commandMaxLength, int valueMaxLength);
bool_t hm10bleTest(int32_t uart);
void hm10blePrintATCommands(int32_t uart);

void initGpios()
{
  gpioInit(GREEN_LED, GPIO_OUTPUT);
  gpioInit(BLUE_LED, GPIO_OUTPUT);
  gpioInit(RED_LED, GPIO_OUTPUT);
  gpioInit(WATER_PIN, GPIO_OUTPUT);
  gpioWrite(GREEN_LED, ON);
  gpioWrite(BLUE_LED, ON);
  gpioWrite(RED_LED, ON);
  gpioWrite(WATER_PIN, OFF);
}

//==================[funcion principal]======================================/

// FUNCION PRINCIPAL, PUNTO DE ENTRADA AL PROGRAMA LUEGO DE ENCENDIDO O RESET.
int main(void)
{

  // ---------- CONFIGURACIONES ------------------------------
  // Inicializar y configurar la plataforma
  boardConfig();
  delay(1);
  tickConfig(1);
  adcConfig(ADC_ENABLE); /* ADC */
  delay(50);
  // Inicializar UART_USB para conectar a la PC
  uartConfig(UART_PC, BAUDRATE);
  delay(50);
  uartWriteString(UART_PC, "UART_PC configurada.\r\n");
  delayInit(&nb_delay, 1000);

  initGpios();
  // Inicializar UART_232 para conectar al modulo bluetooth
  uartConfig(UART_BLUETOOTH, BAUDRATE);
  delay(50);
  uartWriteString(UART_PC, "UART_BLUETOOTH para modulo Bluetooth configurada.\r\n");
  ultrasonicSensorConfig(ULTRASONIC_SENSOR_0, ULTRASONIC_SENSOR_ENABLE);
  delay(50);
  uint8_t data;
  uint8_t dataUart;

  /* Buffer */
  static char uartBuff[10];

  uartWriteString(UART_PC, "Testeto si el modulo esta conectado enviando: AT\r\n");
  if (hm10bleTest(UART_BLUETOOTH))
  {
    uartWriteString(UART_PC, "Modulo conectado correctamente.\r\n");
  }
  else
  {
    uartWriteString(UART_PC, "No funciona.\r\n");
  }

  uint8_t command[100]; // Assuming a maximum command length of 100 bytes
  uint8_t value[100];

  // ---------- REPETIR POR SIEMPRE --------------------------
  while (TRUE)
  {
    if (readCommandAndValue(command, value, sizeof(command), sizeof(value)))
    {
      printf("Received command: %s\r\n", command);
      // Check if command equals "TURN_ON"
      if (strcmp((char *)command, "TURN_ON") == 0)
      {
        printf("Adentro de TURN ON \r\n");
        turnLeds(1);
        gpioWrite(WATER_PIN, ON); // Prendo la bomba
      }
      else if (strcmp((char *)command, "TURN_OFF") == 0)
      {
        printf("Adentro de TURN OFF \r\n");
        turnLeds(0);
        gpioWrite(WATER_PIN, OFF); // Apago la bomba
      }
      else if (strcmp((char *)command, "CH_T") == 0)
      {
        /* esto lo que sea dividir por 50 */
        scan_period = atoi((char *)value);
        delayWrite(&nb_delay, scan_period);
        stdioPrintf(UART_PC, "Cambia el valor de scan_period a: %d \r\n", scan_period);
      }
      else if (strcmp((char *)command, "CH_H") == 0)
      {
        min_humidity = atoi((char *)value);
        stdioPrintf(UART_PC, "Cambia el valor de min_humidity a: %d  \r\n", min_humidity);
      }
      else if (strcmp((char *)command, "CH_D") == 0)
      {
        recipient_height = atoi((char *)value) - 3;
        stdioPrintf(UART_PC, "Cambia el valor de recipient_height a: %d \r\n", recipient_height);
      }
      else
      {
        printf("Received unknown command: %s\r\n", command);
      }
    }
    tickCallbackSet(readSoilOnTick, (void *)CH1);
    delay(50);
  }
  return 0;
}

//==================[definiciones de funciones internas]=====================/

//==================[definiciones de funciones externas]=====================/

bool_t readCommandAndValue(uint8_t *command, uint8_t *value, int commandMaxLength, int valueMaxLength)
{
  uint8_t data;
  int commandIndex = 0;
  int valueIndex = 0;
  int isReadingValue = 0;

  while (1)
  {
    // Read a byte from UART
    if (uartReadByte(UART_BLUETOOTH, &data))
    {
      delay(10);
      // printf("Received: 0x%02X '%c'\r\n", data, (isprint(data) ? data : '.'));
      //  Check for '=' or ';' character
      if (data == '=')
      {
        isReadingValue = 1;
      }
      else if (data == ';')
      {
        // Terminate the strings and break when ';' is encountered
        command[commandIndex] = '\0';
        value[valueIndex] = '\0';
        return 1; // Successfully read a command and value
      }
      else
      {
        // Append the character to the appropriate buffer
        if (isReadingValue)
        {
          if (valueIndex < valueMaxLength - 1)
          {
            value[valueIndex++] = data;
          }
        }
        else
        {
          if (commandIndex < commandMaxLength - 1)
          {
            command[commandIndex++] = data;
          }
        }
      }
    }
    else
    {
      // An error occurred, return false
      return 0;
    }
  }
}

bool_t hm10bleTest(int32_t uart)
{
  uartWriteString(uart, "AT\r\n");
  return waitForReceiveStringOrTimeoutBlocking(uart,
                                               "OK\r\n", strlen("OK\r\n"),
                                               1000);
}

void hm10blePrintATCommands(int32_t uart)
{
  uartWriteString(uart, "AT+HELP?\r\n");
}

void turnLeds(int on)
{
  if (on == 1)
  {
    gpioWrite(RED_LED, OFF);
    gpioWrite(BLUE_LED, OFF);
    gpioWrite(GREEN_LED, OFF);
  }
  else
  {
    gpioWrite(RED_LED, ON);
    gpioWrite(BLUE_LED, ON);
    gpioWrite(GREEN_LED, ON);
  }
}

///==================[fin del archivo]========================================//
