/* Copyright 2014, ChaN
 * Copyright 2016, Matias Marando
 * Copyright 2016, Eric Pernia
 * Copyright 2017, Ericson Estupiñan
 *
 * All rights reserved.
 *
 * This file is part of Workspace.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions are met:
 *
 * 1. Redistributions of source code must retain the above copyright notice,
 *    this list of conditions and the following disclaimer.
 *
 * 2. Redistributions in binary form must reproduce the above copyright notice,
 *    this list of conditions and the following disclaimer in the documentation
 *    and/or other materials provided with the distribution.
 *
 * 3. Neither the name of the copyright holder nor the names of its
 *    contributors may be used to endorse or promote products derived from this
 *    software without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
 * AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
 * IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
 * ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE
 * LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
 * CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
 * SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
 * INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
 * CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
 * ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 * POSSIBILITY OF SUCH DAMAGE.
 *
 */

/*==================[inlcusiones]============================================*/

#include "lab1.h" // <= own header (optional)
#include "ff.h"   // <= Biblioteca FAT FS
#include "sapi.h" // <= sAPI header
#include <string.h>
#include <stdio.h>

/*==================[definiciones y macros]==================================*/

#define FILENAME "Muestras.txt"

/*==================[definiciones de datos internos]=========================*/

static FATFS fs; // <-- FatFs work area needed for each volume
static FIL fp;   // <-- File object needed for each open file
static char uartBuff[10];
static char dateString[20];

/*==================[definiciones de datos externos]=========================*/

/*==================[declaraciones de funciones internas]====================*/

/*==================[declaraciones de funciones externas]====================*/

// FUNCION que se ejecuta cada vezque ocurre un Tick
void diskTickHook(void *ptr);

/*==================[funcion principal]======================================*/

/**
 * C++ version 0.4 char* style "itoa":
 * Written by Lukás Chmela
 * Released under GPLv3.
 */
char *itoa(int value, char *result, int base) {
    // check that the base if valid
    if (base < 2 || base > 36) {
        *result = '\0';
        return result;
    }

    char *ptr = result, *ptr1 = result, tmp_char;
    int tmp_value;

    do {
        tmp_value = value;
        value /= base;
        *ptr++ =
            "zyxwvutsrqponmlkjihgfedcba9876543210123456789abcdefghijklmnopqrst"
            "uvwxyz"[35 + (tmp_value - value * base)];
    } while (value);

    // Apply negative sign
    if (tmp_value < 0)
        *ptr++ = '-';
    *ptr-- = '\0';
    while (ptr1 < ptr) {
        tmp_char = *ptr;
        *ptr-- = *ptr1;
        *ptr1++ = tmp_char;
    }
    return result;
}

void stringAdd(char *destBuffer, char *srcBuffer) {
    sprintf(destBuffer, "%s%s", destBuffer, srcBuffer);
}

/* Enviar fecha y hora en formato "DD/MM/YYYY, HH:MM:SS" */
void loadDateString(rtc_t *rtc) {

    bzero(dateString, 20);
    /* Conversion de entero a ascii con base decimal */
    itoa((int)(rtc->year), (char *)uartBuff, 10); /* 10 significa decimal */
    /* Envio el año */
    if ((rtc->year) < 10)
        stringAdd(dateString, "0");

    stringAdd(dateString, uartBuff);
    stringAdd(dateString, "/");

    /* Conversion de entero a ascii con base decimal */
    itoa((int)(rtc->month), (char *)uartBuff, 10); /* 10 significa decimal */
    /* Envio el mes */
    if ((rtc->month) < 10)
        stringAdd(dateString, "0");

    stringAdd(dateString, uartBuff);
    stringAdd(dateString, "/");

    /* Conversion de entero a ascii con base decimal */
    itoa((int)(rtc->mday), (char *)uartBuff, 10); /* 10 significa decimal */
    /* Envio el dia */
    if ((rtc->mday) < 10)
        stringAdd(dateString, "0");

    stringAdd(dateString, uartBuff);
    stringAdd(dateString, "_");

    /* Conversion de entero a ascii con base decimal */
    itoa((int)(rtc->hour), (char *)uartBuff, 10); /* 10 significa decimal */
    /* Envio la hora */
    if ((rtc->hour) < 10)
        stringAdd(dateString, "0");

    stringAdd(dateString, uartBuff);
    stringAdd(dateString, ":");

    /* Conversion de entero a ascii con base decimal */
    itoa((int)(rtc->min), (char *)uartBuff, 10); /* 10 significa decimal */
    /* Envio los minutos */
    // uartBuff[2] = 0;    /* NULL */
    if ((rtc->min) < 10)
        stringAdd(dateString, "0");

    stringAdd(dateString, uartBuff);
    stringAdd(dateString, ":");

    /* Conversion de entero a ascii con base decimal */
    itoa((int)(rtc->sec), (char *)uartBuff, 10); /* 10 significa decimal */
    /* Envio los segundos */
    if ((rtc->sec) < 10)
        stringAdd(dateString, "0");

    stringAdd(dateString, uartBuff);
}

// FUNCION PRINCIPAL, PUNTO DE ENTRADA AL PROGRAMA LUEGO DE ENCENDIDO O RESET.
int main(void) {

    // ---------- CONFIGURACIONES ------------------------------
    // Inicializar y configurar la plataforma
    boardConfig();

    /* Estructura RTC */
    rtc_t rtc;
    bool_t val = 0;

    val = rtcRead(&rtc);

    // rtc.year = 2018;
    // rtc.month = 7;
    // rtc.mday = 04;
    // rtc.wday = 2;
    // rtc.hour = 17;
    // rtc.min = 34;
    // rtc.sec = 0;

    /* Inicializar RTC */
    // val = rtcInit(&rtc);

    delay_t delay1s;
    delay_t delay5s;
    delayConfig(&delay1s, 1000);
    delayConfig(&delay5s, 5000);

    /* Inicializar UART_USB a 115200 baudios */
    uartConfig(UART_USB, 115200);

    // SPI configuration
    spiConfig(SPI0);

    // Inicializar el conteo de Ticks con resolucion de 10ms,
    // con tickHook diskTickHook
    tickConfig(1);
    tickCallbackSet(diskTickHook, NULL);

    /* Inicializar AnalogIO */
    /* Posibles configuraciones:
     *    ADC_ENABLE,  ADC_DISABLE,
     *    ADC_ENABLE,  ADC_DISABLE,
     */
    adcConfig(ADC_ENABLE); /* ADC */

    char msgStart[] = "Start Programa...\r\n";
    uartWriteString(UART_USB, msgStart); /* Envía "Hola de nuevo\r\n" */

    // val = rtcRead(&rtc);
    // showDateAndTime(&rtc);

    // ------ PROGRAMA QUE ESCRIBE EN LA SD FROM UART -------

    UINT nbytes;

    // Create/open a file, then write a string and close it

    uint8_t caracter = 'x';
    uint8_t i = 0;
    uint8_t buff[50];

    // Give a work area to the default drive
    if (f_mount(&fs, "", 0) != FR_OK) {
        // If this fails, it means that the function could
        // not register a file system object.
        // Check whether the SD card is correctly connected
    }

    while (TRUE) {
        /* Si recibe un byte de la UART_USB lo guardarlo en la variable
         * caracter. */
        // if (uartReadByte(UART_USB, &caracter)) {
        /* Se reenvíael dato a la UART_USB realizando un eco de lo que
         * llega */

        if (delayRead(&delay5s)) {

            /* Leo la Entrada Analogica AI0 - ADC0 CH1 */
            uint16_t muestra1 = adcRead(CH1);
            uint16_t muestra2 = adcRead(CH2);
            uint16_t muestra3 = adcRead(CH3);

            val = rtcRead(&rtc);
            loadDateString(&rtc);
            sprintf(buff, "%d;%d;%d;%s;\r\n", muestra1, muestra2, muestra3,
                    dateString);

            if (f_open(&fp, FILENAME, FA_WRITE | FA_OPEN_APPEND) == FR_OK) {
                uint8_t len = strlen(buff);
                f_write(&fp, buff, len, &nbytes);
                gpioWrite(LEDR, OFF);
            } else {
                // Turn ON LEDR if the write operation was fail
                gpioWrite(LEDR, ON);
            }
            
            f_close(&fp);
            // if (caracter == 'x') {
            //    break;
            //}
            uartWriteString(UART_USB, buff);
            // }
        }
    }

    char msgEnd[] = "End Programa...\r\n";
    uartWriteString(UART_USB, msgEnd); /* Envía "Hola de nuevo\r\n" */

    while (TRUE) {
    }
    // NO DEBE LLEGAR NUNCA AQUI, debido a que a este programa se ejecuta
    // directamenteno sobre un microcontroladore y no es llamado/ por ningun
    // Sistema Operativo, como en el caso de un programa para PC.
    return 0;
}

/*==================[definiciones de funciones internas]=====================*/

/*==================[definiciones de funciones externas]=====================*/

// FUNCION que se ejecuta cada vezque ocurre un Tick
void diskTickHook(void *ptr) {
    disk_timerproc(); // Disk timer process
}

/*==================[fin del archivo]========================================*/
