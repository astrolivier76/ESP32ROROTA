// --- Fichier: config.h ---
#ifndef CONFIG_H
#define CONFIG_H

#include <IPAddress.h>

// ==========================================
// 1. PARAMÈTRES RÉSEAU (WI-FI & mDNS)
// ==========================================
const char* ssid     = "maison_casba";
const char* password = "casandepa76*";

// Port du serveur TCP (à renseigner dans HW VSP3)
const int portTCP = 8888;

// Nom d'accès mDNS (ex: http://toit-astro.local)
const char* mdns_name = "esp32-toit";

// ==========================================
// 2. AFFECTATION DES BROCHES (PINS ESP32)
// ==========================================
// --- Relais (vers module optocoupleur PC817) ---
const int pinOpenRelay   = 25; // Impulsion d'ouverture du toit
const int pinCloseRelay  = 26; // Impulsion de fermeture du toit

// --- Capteurs ---
const int pinOpenedSensor = 13; // Fin de course (NC): Toit Ouvert
const int pinClosedSensor = 14; // Fin de course (NC): Toit Fermé
const int pinSafeSensor   = 23; // Infrarouge OMRON (NPN): Signal de parking

// ==========================================
// 3. LOGIQUE MATÉRIELLE
// ==========================================
// Mettre LOW car le capteur NPN ferme le circuit à la masse quand le faisceau est aligné
const int ETAT_SECURISE = LOW; 

// ==========================================
// 4. TIMEOUT MATÉRIEL
// ==========================================
#define WDT_TIMEOUT 3
#define TIMEOUT_MOUVEMENT 15000 // Temps maximum autorisé pour une manœuvre

#endif
