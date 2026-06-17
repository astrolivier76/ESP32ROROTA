// --- Fichier: config.h ---
#ifndef CONFIG_H
#define CONFIG_H

#include <IPAddress.h>

// ==========================================
// 1. PARAMÈTRES RÉSEAU (WI-FI & IP FIXE)
// ==========================================
const char* ssid     = "VOTRE_NOM_DE_RESEAU_WIFI";
const char* password = "VOTRE_MOT_DE_PASSE_WIFI";

// Port du serveur TCP (à renseigner dans HW VSP3)
const int portTCP = 8888;

// --- Configuration de l'IP Statique ---
IPAddress local_IP(192, 168, 0, 198);   // L'adresse IP fixe attribuée à l'ESP32
IPAddress gateway(192, 168, 0, 1);      // La passerelle (Box / Routeur)
IPAddress subnet(255, 255, 255, 0);     // Le masque de sous-réseau

// DNS Google
IPAddress primaryDNS(8, 8, 8, 8);      
IPAddress secondaryDNS(8, 8, 4, 4);    


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
#define TIMEOUT_MOUVEMENT 25000 // Temps maximum autorisé pour une manœuvre

#endif
