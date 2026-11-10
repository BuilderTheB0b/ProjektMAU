/**
 * @file mesh_tasks.h
 * @brief Deklaration av FreeRTOS-uppgifter för ESP-MESH-kommunikation.
 *
 * Filen innehåller funktionsdeklarationer för sändning och
 * mottagning av meddelanden i mesh-nätverket.
 *
 * mesh_tx_task ansvarar för att skicka heartbeat- och larmmeddelanden
 * från en nod till root-noden.
 *
 * mesh_rx_task ansvarar för att ta emot meddelanden från andra noder
 * samt beräkna kommunikationslatens baserat på tidsstämplar.
 */
#ifndef MESH_TASKS_H
#define MESH_TASKS_H

void mesh_tx_task(void *arg);
void mesh_rx_task(void *arg);

#endif