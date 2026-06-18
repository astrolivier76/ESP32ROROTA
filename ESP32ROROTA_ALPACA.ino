// ==========================================================
// Motorisation Observatoire ESP32 + RRCI ASCOM 1.3.1 + WEB OTA + ALPACA
// Architecture Fail-Safe matérielle via IPX800
// ==========================================================

#include <WiFi.h>
#include <WebServer.h>
#include <ESPmDNS.h>      // Ajout de la bibliothèque mDNS
#include <esp_task_wdt.h>
#include <Update.h>
#include <ArduinoJson.h>
#include "config.h"

// ===================== RÉSEAU =============================
WiFiServer server(portTCP); // Port ASCOM RRCI (ex: 8888)
WebServer serverWeb(80);    // Port HTTP Interface, OTA & Alpaca
WiFiClient client;

char networkIn[64];
uint8_t netIndex = 0;

// ===================== ETATS ===============================
enum EtatToit {
  ETAT_INCONNU,
  ETAT_FERME,
  ETAT_OUVERT,
  ETAT_EN_OUVERTURE,
  ETAT_EN_FERMETURE,
  ETAT_ERREUR,
  ETAT_SAFE_BLOCKED
};

enum CommandeToit {
  CMD_AUCUNE,
  CMD_OUVRIR,
  CMD_FERMER,
  CMD_RESET
};

volatile EtatToit etatCourant = ETAT_INCONNU;
QueueHandle_t fileCommandes;

// ===================== TIMERS ==============================
unsigned long chronoMouvement = 0;
unsigned long timerRelais = 0;

// ================ VARIABLES ALPACA =========================
bool isAlpacaConnected = false;

// ===================== CAPTEURS ============================
struct Capteur {
  int pin;
  bool etatStable;
  bool dernier;
  unsigned long t;
  unsigned long debounce;
};

Capteur capteurOuvert  = {pinOpenedSensor, HIGH, HIGH, 0, 50};
Capteur capteurFerme   = {pinClosedSensor, HIGH, HIGH, 0, 50};
Capteur capteurSafe    = {pinSafeSensor, HIGH, HIGH, 0, 500};

#define SAFE_OK LOW
#define SAFE_BLOCKED HIGH

// ===================== CAPTEUR =============================
void lire(Capteur &c) {
  bool v = digitalRead(c.pin);
  if (v != c.dernier) c.t = millis();
  if (millis() - c.t > c.debounce) c.etatStable = v;
  c.dernier = v;
}

// ===================== STOP TOTAL ==========================
void stopUrgence() {
  digitalWrite(pinOpenRelay, LOW);
  digitalWrite(pinCloseRelay, LOW);
}

// ===================== FSM ULTRA STRICTE ===================
void transition(EtatToit e) {
  // PRIORITÉ ABSOLUE SAFETY
  if (e == ETAT_SAFE_BLOCKED || e == ETAT_ERREUR) {
    stopUrgence();
    etatCourant = e;
    return;
  }
  // Verrouillage total si SAFE BLOCKED
  if (etatCourant == ETAT_SAFE_BLOCKED) return;

  switch (etatCourant) {
    case ETAT_INCONNU:
      etatCourant = e;
      break;
    case ETAT_FERME:
      if (e == ETAT_EN_OUVERTURE) etatCourant = e;
      break;
    case ETAT_OUVERT:
      if (e == ETAT_EN_FERMETURE) etatCourant = e;
      break;
    case ETAT_EN_OUVERTURE:
      if (e == ETAT_OUVERT || e == ETAT_ERREUR) etatCourant = e;
      break;
    case ETAT_EN_FERMETURE:
      if (e == ETAT_FERME || e == ETAT_ERREUR) etatCourant = e;
      break;
    default:
      break;
  }
}

// ===================== MAJ CAPTEURS ========================
void majCapteurs() {
  
  // 1. AUTO-RÉARMEMENT : Le télescope vient de se re-parquer
  if (etatCourant == ETAT_SAFE_BLOCKED && capteurSafe.etatStable == SAFE_OK) {
    etatCourant = ETAT_INCONNU; // On lève l'erreur, le système va se recaler au prochain cycle
    return;
  }

  // 2. PRIORITÉ ABSOLUE : SÉCURITÉ IPX800
  if (capteurSafe.etatStable == SAFE_BLOCKED) {
    // TOLÉRANCE : On ignore l'alerte uniquement si le toit est COMPLÈTEMENT ouvert
    if (etatCourant != ETAT_OUVERT) {
      if (etatCourant != ETAT_SAFE_BLOCKED) {
        stopUrgence();
        etatCourant = ETAT_SAFE_BLOCKED;
      }
    }
    return; // On stoppe l'évaluation des autres capteurs car la sécurité prime ou on est en tolérance
  }

  // 3. PARADOXE MATÉRIEL : Les deux capteurs fin de course sont à LOW
  if (capteurOuvert.etatStable == LOW && capteurFerme.etatStable == LOW) {
    if (etatCourant != ETAT_ERREUR) {
      transition(ETAT_ERREUR);
    }
    return;
  }

  // 4. LECTURE NORMALE : Mise à jour de l'état
  if (capteurOuvert.etatStable == LOW) transition(ETAT_OUVERT);
  else if (capteurFerme.etatStable == LOW) transition(ETAT_FERME);

  // 5. PERTE DE CAPTEUR ANORMALE (Toit physiquement poussé ou capteur arraché)
  if (etatCourant == ETAT_OUVERT && capteurOuvert.etatStable == HIGH) {
    transition(ETAT_ERREUR);
  }
  if (etatCourant == ETAT_FERME && capteurFerme.etatStable == HIGH) {
    transition(ETAT_ERREUR);
  }
}

// ===================== EXEC COMMAND ========================
void exec(CommandeToit c) {
  if (etatCourant == ETAT_SAFE_BLOCKED || etatCourant == ETAT_ERREUR) {
    if (c == CMD_RESET) {
      stopUrgence();
      etatCourant = ETAT_INCONNU;
    }
    return;
  }

  if (c == CMD_RESET) {
    stopUrgence();
    etatCourant = ETAT_INCONNU;
    return;
  }

  if (c == CMD_OUVRIR && etatCourant == ETAT_FERME) {
    digitalWrite(pinCloseRelay, LOW);
    digitalWrite(pinOpenRelay, HIGH);
    chronoMouvement = millis();
    timerRelais = millis();
    transition(ETAT_EN_OUVERTURE);
  }

  if (c == CMD_FERMER && etatCourant == ETAT_OUVERT && capteurSafe.etatStable == SAFE_OK) {
    digitalWrite(pinOpenRelay, LOW);
    digitalWrite(pinCloseRelay, HIGH);
    chronoMouvement = millis();
    timerRelais = millis();
    transition(ETAT_EN_FERMETURE);
  }
}

// ===================== TIMEOUT =============================
void checkTimeout() {
  if ((etatCourant == ETAT_EN_OUVERTURE || etatCourant == ETAT_EN_FERMETURE) &&
      millis() - chronoMouvement > TIMEOUT_MOUVEMENT) {
    stopUrgence();
    etatCourant = ETAT_ERREUR;
  }
}

void majRelais() {
  if (millis() - timerRelais > 1000) {
    digitalWrite(pinOpenRelay, LOW);
    digitalWrite(pinCloseRelay, LOW);
  }
}

// ===================== ASCOM (RRCI V1.3.1) =================
void ascom(String cmd) {
  cmd.trim();
  String originalCmd = cmd; 
  cmd.toLowerCase();

  // --- TRACEUR POUR LE DÉBOGAGE ---
  if (cmd != "status" && cmd != "ping") { 
    Serial.print("[ASCOM TCP] <-- NINA a dit : ");
    Serial.println(originalCmd);
  }

  if (cmd == "ping") {
    client.print("PONG#");
  }
  else if (cmd == "status") {
    String resp = "STATE:";
    if (etatCourant == ETAT_OUVERT) resp += "OPEN;";
    else if (etatCourant == ETAT_FERME) resp += "CLOSED;";
    else if (etatCourant == ETAT_EN_OUVERTURE) resp += "OPENING;";
    else if (etatCourant == ETAT_EN_FERMETURE) resp += "CLOSING;";
    else if (etatCourant == ETAT_ERREUR || etatCourant == ETAT_SAFE_BLOCKED) resp += "ERROR;";
    else resp += "IDLE;";

    resp += (capteurSafe.etatStable == SAFE_OK) ? "SAFE;" : "UNSAFE;";

    if (etatCourant == ETAT_EN_OUVERTURE || etatCourant == ETAT_EN_FERMETURE) resp += "MOVING#";
    else resp += "IDLE#";

    client.print(resp);
  }
  else if (cmd == "open") {
    CommandeToit c = CMD_OUVRIR;
    xQueueSend(fileCommandes, &c, 0);
    client.print("OK:open#");
  }
  else if (cmd == "close") {
    CommandeToit c = CMD_FERMER;
    xQueueSend(fileCommandes, &c, 0);
    client.print("OK:close#");
  }
  else if (cmd == "abort") {
    CommandeToit c = CMD_RESET;
    xQueueSend(fileCommandes, &c, 0);
    client.print("OK:abort#");
  }
  else if (cmd.startsWith("setsafe")) {
    client.print("OK:setsafe#");
  }
  else if (cmd.startsWith("setmotion")) {
    client.print("OK:setmotion#");
  }
  else if (cmd.startsWith("setmode")) {
    client.print("OK:setmode#");
  }
  else if (cmd == "getpulsecount") {
    client.print("PULSES:0#");
  }
  else if (cmd == "resetpulse") {
    client.print("OK#");
  }
  else {
    client.print("ERR:");
    client.print(originalCmd);
    client.print("#");
  }
}

// ===================== TACHE SECURITE (Core 1) =============
void codeSafe(void *p) {
  
  // Configuration du Watchdog matériel (Compatible Core ESP32 v3.0+)
  esp_task_wdt_config_t twdt_config = {
      .timeout_ms = WDT_TIMEOUT * 1000,
      .idle_core_mask = (1 << portNUM_PROCESSORS) - 1,
      .trigger_panic = true,
  };
  esp_task_wdt_init(&twdt_config);
  esp_task_wdt_add(NULL);
  
  CommandeToit c;
  for (;;) {
    lire(capteurOuvert);
    lire(capteurFerme);
    lire(capteurSafe);

    majCapteurs();

    if (xQueueReceive(fileCommandes, &c, 0) == pdPASS) exec(c);

    majRelais();
    checkTimeout();

    // On réinitialise le Watchdog à chaque cycle pour prouver que tout va bien
    esp_task_wdt_reset();
    vTaskDelay(10 / portTICK_PERIOD_MS);
  }
}

// ===================== FONCTION GLOBALE ALPACA =====================
// Cette fonction lit le numéro de transaction NINA pour toutes les routes
uint32_t getClientID() {
  if (serverWeb.hasArg("ClientTransactionID")) {
    return serverWeb.arg("ClientTransactionID").toInt();
  }
  return 0;
}

// ===================== TACHE NET & WEB (Core 0) ============
void codeNet(void *p) {
  WiFi.begin(ssid, password);

  // --- Attente connexion initiale ---
  while (WiFi.status() != WL_CONNECTED) {
    vTaskDelay(500 / portTICK_PERIOD_MS);
    Serial.print(".");
  }
  Serial.println("");
  Serial.print("Connecté au Wi-Fi ! IP : ");
  Serial.println(WiFi.localIP());

  // --- INITIALISATION mDNS ---
  if (MDNS.begin(mdns_name)) {
    Serial.println("Serveur mDNS démarre !");
    MDNS.addService("http", "tcp", 80);
  }

  // API locale pour renvoyer l'état en texte au JavaScript (AJAX)
  serverWeb.on("/api/status", HTTP_GET, []() {
    String etatStr = "INCONNU";
    switch(etatCourant) {
      case ETAT_FERME:          etatStr = "FERMÉ"; break;
      case ETAT_OUVERT:         etatStr = "OUVERT"; break;
      case ETAT_EN_OUVERTURE:   etatStr = "EN OUVERTURE..."; break;
      case ETAT_EN_FERMETURE:   etatStr = "EN FERMETURE..."; break;
      case ETAT_ERREUR:         etatStr = "ERREUR CAPTEUR / TIMEOUT"; break;
      case ETAT_SAFE_BLOCKED:   etatStr = "SÉCURITÉ TOIT BLOQUÉE (IPX800)"; break;
      default:                  etatStr = "INCONNU"; break;
    }
    serverWeb.send(200, "text/plain", etatStr);
  });

  // ===================== ASCOM ALPACA : MANAGEMENT API =====================
  serverWeb.on("/management/apiversions", HTTP_GET, []() {
    serverWeb.sendHeader("Connection", "close");
    serverWeb.send(200, "application/json", "{\"Value\": [1], \"ClientTransactionID\": 0, \"ServerTransactionID\": 1, \"ErrorNumber\": 0, \"ErrorMessage\": \"\"}");
  });

  serverWeb.on("/management/v1/configureddevices", HTTP_GET, []() {
    JsonDocument doc; 
    JsonArray value = doc["Value"].to<JsonArray>();
    JsonObject dome = value.add<JsonObject>();
    
    dome["DeviceName"] = "ESP32 Observatoire";
    dome["DeviceType"] = "Dome";
    dome["DeviceNumber"] = 0;
    dome["UniqueID"] = "ESP32-TOIT-123456";

    doc["ClientTransactionID"] = getClientID();
    doc["ServerTransactionID"] = 1;
    doc["ErrorNumber"] = 0;
    doc["ErrorMessage"] = "";

    String output;
    serializeJson(doc, output);
    serverWeb.sendHeader("Connection", "close");
    serverWeb.send(200, "application/json", output);
  });

  serverWeb.on("/management/v1/description", HTTP_GET, []() {
    serverWeb.send(200, "application/json", "{\"Value\":{\"ServerName\":\"ESP32\",\"Manufacturer\":\"ToitAstro\",\"ManufacturerVersion\":\"1.0\",\"Location\":\"Observatoire\"},\"ClientTransactionID\":0,\"ServerTransactionID\":1,\"ErrorNumber\":0,\"ErrorMessage\":\"\"}");
  });

  // ===================== ASCOM ALPACA : BASE API (OBLIGATOIRE) =====================
  serverWeb.on("/api/v1/dome/0/name", HTTP_GET, []() { serverWeb.send(200, "application/json", "{\"Value\":\"ESP32 Observatoire\",\"ClientTransactionID\":0,\"ServerTransactionID\":1,\"ErrorNumber\":0,\"ErrorMessage\":\"\"}"); });
  serverWeb.on("/api/v1/dome/0/description", HTTP_GET, []() { serverWeb.send(200, "application/json", "{\"Value\":\"Toit ROR ESP32\",\"ClientTransactionID\":0,\"ServerTransactionID\":1,\"ErrorNumber\":0,\"ErrorMessage\":\"\"}"); });
  serverWeb.on("/api/v1/dome/0/driverinfo", HTTP_GET, []() { serverWeb.send(200, "application/json", "{\"Value\":\"ESP32ROROTA_Alpaca v1.0\",\"ClientTransactionID\":0,\"ServerTransactionID\":1,\"ErrorNumber\":0,\"ErrorMessage\":\"\"}"); });
  serverWeb.on("/api/v1/dome/0/driverversion", HTTP_GET, []() { serverWeb.send(200, "application/json", "{\"Value\":\"1.0\",\"ClientTransactionID\":0,\"ServerTransactionID\":1,\"ErrorNumber\":0,\"ErrorMessage\":\"\"}"); });
  serverWeb.on("/api/v1/dome/0/interfaceversion", HTTP_GET, []() { serverWeb.send(200, "application/json", "{\"Value\":1,\"ClientTransactionID\":0,\"ServerTransactionID\":1,\"ErrorNumber\":0,\"ErrorMessage\":\"\"}"); });
  serverWeb.on("/api/v1/dome/0/supportedactions", HTTP_GET, []() { serverWeb.send(200, "application/json", "{\"Value\":[],\"ClientTransactionID\":0,\"ServerTransactionID\":1,\"ErrorNumber\":0,\"ErrorMessage\":\"\"}"); });

  // ===================== ASCOM ALPACA : DOME API (PROPRIÉTÉS) =====================
  serverWeb.on("/api/v1/dome/0/connected", HTTP_GET, []() {
    JsonDocument doc;
    doc["Value"] = isAlpacaConnected;
    doc["ClientTransactionID"] = getClientID();
    doc["ServerTransactionID"] = 1;
    doc["ErrorNumber"] = 0;
    doc["ErrorMessage"] = "";
    String out; serializeJson(doc, out);
    serverWeb.send(200, "application/json", out);
  });

  serverWeb.on("/api/v1/dome/0/connected", HTTP_PUT, []() {
    if (serverWeb.hasArg("Connected")) {
      isAlpacaConnected = (serverWeb.arg("Connected").equalsIgnoreCase("True"));
    }
    JsonDocument doc;
    doc["ClientTransactionID"] = getClientID();
    doc["ServerTransactionID"] = 1;
    doc["ErrorNumber"] = 0;
    doc["ErrorMessage"] = "";
    String out; serializeJson(doc, out);
    serverWeb.send(200, "application/json", out);
  });

  serverWeb.on("/api/v1/dome/0/shutterstatus", HTTP_GET, []() {
    int alpacaState = 4; // Par défaut : 4 = Erreur
    if (etatCourant == ETAT_OUVERT) alpacaState = 0;
    else if (etatCourant == ETAT_FERME) alpacaState = 1;
    else if (etatCourant == ETAT_EN_OUVERTURE) alpacaState = 2;
    else if (etatCourant == ETAT_EN_FERMETURE) alpacaState = 3;
    
    JsonDocument doc;
    doc["Value"] = alpacaState;
    doc["ClientTransactionID"] = getClientID();
    doc["ServerTransactionID"] = 1;
    doc["ErrorNumber"] = 0;
    doc["ErrorMessage"] = "";
    String out; serializeJson(doc, out);
    serverWeb.send(200, "application/json", out);
  });

  // --- Autorisations du Dôme ---
  serverWeb.on("/api/v1/dome/0/canfindhome", HTTP_GET, []() { serverWeb.send(200, "application/json", "{\"Value\":false,\"ErrorNumber\":0}"); });
  serverWeb.on("/api/v1/dome/0/canpark", HTTP_GET, []() { serverWeb.send(200, "application/json", "{\"Value\":false,\"ErrorNumber\":0}"); });
  serverWeb.on("/api/v1/dome/0/cansetshutter", HTTP_GET, []() { serverWeb.send(200, "application/json", "{\"Value\":true,\"ErrorNumber\":0}"); });
  serverWeb.on("/api/v1/dome/0/cansetaltitude", HTTP_GET, []() { serverWeb.send(200, "application/json", "{\"Value\":false,\"ErrorNumber\":0}"); });
  serverWeb.on("/api/v1/dome/0/cansetazimuth", HTTP_GET, []() { serverWeb.send(200, "application/json", "{\"Value\":false,\"ErrorNumber\":0}"); });
  serverWeb.on("/api/v1/dome/0/canslave", HTTP_GET, []() { serverWeb.send(200, "application/json", "{\"Value\":false,\"ErrorNumber\":0}"); });
  serverWeb.on("/api/v1/dome/0/cansyncazimuth", HTTP_GET, []() { serverWeb.send(200, "application/json", "{\"Value\":false,\"ErrorNumber\":0}"); });
  serverWeb.on("/api/v1/dome/0/cansetpark", HTTP_GET, []() { serverWeb.send(200, "application/json", "{\"Value\":false,\"ErrorNumber\":0}"); });

  // --- Capteurs & État du Dôme ---
  serverWeb.on("/api/v1/dome/0/slewing", HTTP_GET, []() {
    bool isMoving = (etatCourant == ETAT_EN_OUVERTURE || etatCourant == ETAT_EN_FERMETURE);
    String rep = "{\"Value\":" + String(isMoving ? "true" : "false") + ",\"ClientTransactionID\":0,\"ServerTransactionID\":1,\"ErrorNumber\":0,\"ErrorMessage\":\"\"}";
    serverWeb.send(200, "application/json", rep);
  });
  
  serverWeb.on("/api/v1/dome/0/athome", HTTP_GET, []() { serverWeb.send(200, "application/json", "{\"Value\":false,\"ClientTransactionID\":0,\"ServerTransactionID\":1,\"ErrorNumber\":0,\"ErrorMessage\":\"\"}"); });
  
  serverWeb.on("/api/v1/dome/0/atpark", HTTP_GET, []() {
    bool isParked = (etatCourant == ETAT_FERME);
    String rep = "{\"Value\":" + String(isParked ? "true" : "false") + ",\"ClientTransactionID\":0,\"ServerTransactionID\":1,\"ErrorNumber\":0,\"ErrorMessage\":\"\"}";
    serverWeb.send(200, "application/json", rep);
  });

  serverWeb.on("/api/v1/dome/0/slaved", HTTP_GET, []() { serverWeb.send(200, "application/json", "{\"Value\":false,\"ClientTransactionID\":0,\"ServerTransactionID\":1,\"ErrorNumber\":0,\"ErrorMessage\":\"\"}"); });
  serverWeb.on("/api/v1/dome/0/altitude", HTTP_GET, []() { serverWeb.send(200, "application/json", "{\"Value\":0,\"ErrorNumber\":0}"); });
  serverWeb.on("/api/v1/dome/0/azimuth", HTTP_GET, []() { serverWeb.send(200, "application/json", "{\"Value\":0,\"ErrorNumber\":0}"); });

  // ===================== ASCOM ALPACA : DOME API (ACTIONS VALIDES) =====================
  serverWeb.on("/api/v1/dome/0/openshutter", HTTP_PUT, []() {
    CommandeToit c = CMD_OUVRIR;
    xQueueSend(fileCommandes, &c, 0);
    serverWeb.send(200, "application/json", "{\"ErrorNumber\":0,\"ErrorMessage\":\"\"}");
  });

  serverWeb.on("/api/v1/dome/0/closeshutter", HTTP_PUT, []() {
    CommandeToit c = CMD_FERMER;
    xQueueSend(fileCommandes, &c, 0);
    serverWeb.send(200, "application/json", "{\"ErrorNumber\":0,\"ErrorMessage\":\"\"}");
  });

  serverWeb.on("/api/v1/dome/0/abortslew", HTTP_PUT, []() {
    CommandeToit c = CMD_RESET;
    xQueueSend(fileCommandes, &c, 0);
    serverWeb.send(200, "application/json", "{\"ErrorNumber\":0,\"ErrorMessage\":\"\"}");
  });

  // ===================== ASCOM ALPACA : DOME API (ACTIONS INVALIDES) =====================
  serverWeb.on("/api/v1/dome/0/slaved", HTTP_PUT, []() { serverWeb.send(200, "application/json", "{\"ClientTransactionID\":0,\"ServerTransactionID\":1,\"ErrorNumber\":1024,\"ErrorMessage\":\"Non applicable\"}"); });
  serverWeb.on("/api/v1/dome/0/setpark", HTTP_PUT, []() { serverWeb.send(200, "application/json", "{\"ClientTransactionID\":0,\"ServerTransactionID\":1,\"ErrorNumber\":1024,\"ErrorMessage\":\"Non applicable\"}"); });
  serverWeb.on("/api/v1/dome/0/park", HTTP_PUT, []() { serverWeb.send(200, "application/json", "{\"ClientTransactionID\":0,\"ServerTransactionID\":1,\"ErrorNumber\":1024,\"ErrorMessage\":\"Utilisez CloseShutter\"}"); });
  serverWeb.on("/api/v1/dome/0/findhome", HTTP_PUT, []() { serverWeb.send(200, "application/json", "{\"ClientTransactionID\":0,\"ServerTransactionID\":1,\"ErrorNumber\":1024,\"ErrorMessage\":\"Non applicable\"}"); });
  serverWeb.on("/api/v1/dome/0/slewtoazimuth", HTTP_PUT, []() { serverWeb.send(200, "application/json", "{\"ClientTransactionID\":0,\"ServerTransactionID\":1,\"ErrorNumber\":1024,\"ErrorMessage\":\"Coupole non rotative\"}"); });
  serverWeb.on("/api/v1/dome/0/syncazimuth", HTTP_PUT, []() { serverWeb.send(200, "application/json", "{\"ClientTransactionID\":0,\"ServerTransactionID\":1,\"ErrorNumber\":1024,\"ErrorMessage\":\"Coupole non rotative\"}"); });
  serverWeb.on("/api/v1/dome/0/slewtoaltitude", HTTP_PUT, []() { serverWeb.send(200, "application/json", "{\"ClientTransactionID\":0,\"ServerTransactionID\":1,\"ErrorNumber\":1024,\"ErrorMessage\":\"Cimier non ajustable\"}"); });

  // ===================== Page d'accueil principale (AJAX) =====================
  serverWeb.on("/", HTTP_GET, []() {
    serverWeb.sendHeader("Connection", "close");
    String html = R"rawliteral(
    <!DOCTYPE html>
    <html>
    <head>
      <meta charset='UTF-8'>
      <meta name='viewport' content='width=device-width, initial-scale=1'>
      <style>
        body {background:#111; color:#eee; text-align:center; padding:30px; font-family:sans-serif;}
        .card {background:#222; padding:30px; border-radius:10px; display:inline-block; border:1px solid #444; width:80%; max-width:500px;}
        .btn {padding:12px 24px; margin:10px; color:#fff; border:none; border-radius:6px; font-weight:bold; cursor:pointer; font-size:16px; text-decoration:none; display:inline-block;}
        .btn-open {background:#2f855a;} .btn-open:hover {background:#276749;}
        .btn-close {background:#c53030;} .btn-close:hover {background:#9b2c2c;}
        .btn-reset {background:#dd6b20;} .btn-reset:hover {background:#c05621;}
        .btn-ota {background:#4a5568;} .btn-ota:hover {background:#2d3748;}
        #badge-etat {padding:8px 15px; border-radius:20px; font-weight:bold; background:#4a5568; display:inline-block; margin-top:15px;}
        .moving {background:#dd6b20 !important; color:#fff;}
        .open {background:#38a169 !important; color:#fff;}
        .close {background:#e53e3e !important; color:#fff;}
      </style>
    </head>
    <body>
      <div class="card">
        <h2>Contrôle de l'Abri Astro</h2>
        <hr style="border-color:#444;">
        
        <p>État actuel du toit :</p>
        <div id="badge-etat">CHARGEMENT...</div>
        
        <br><br>
        <button onclick="envoyerCommande('/open')" class="btn btn-open">Ouvrir le Toit</button>
        <button onclick="envoyerCommande('/close')" class="btn btn-close">Fermer le Toit</button><br>
        <button onclick="envoyerCommande('/reset')" class="btn btn-reset">Acquitter Erreur (Reset)</button>
        <br><br>
        <a href="/update" class="btn btn-ota" style="font-size:12px;">Mise à jour Firmware (OTA)</a>
      </div>

      <script>
        function rafraichirEtat() {
          var xhr = new XMLHttpRequest();
          xhr.open("GET", "/api/status", true);
          xhr.onload = function() {
            if (xhr.status === 200) {
              var etat = xhr.responseText;
              var badge = document.getElementById("badge-etat");
              badge.innerHTML = etat;
              
              badge.className = ""; 
              if(etat.includes("OUVERT") && !etat.includes("EN")) badge.classList.add("open");
              else if(etat.includes("FERMÉ") && !etat.includes("EN")) badge.classList.add("close");
              else if(etat.includes("EN")) badge.classList.add("moving");
              else badge.classList.add("moving"); 
            }
          };
          xhr.send();
        }

        function envoyerCommande(url) {
          var xhr = new XMLHttpRequest();
          xhr.open("GET", url, true);
          xhr.send();
          setTimeout(rafraichirEtat, 200);
        }

        rafraichirEtat();
        setInterval(rafraichirEtat, 2000);
      </script>
    </body>
    </html>
    )rawliteral";
    serverWeb.send(200, "text/html", html);
  });

  serverWeb.on("/open", []() {
    Serial.println("[WEB] --> Demande d'OUVERTURE reçue");
    CommandeToit c = CMD_OUVRIR;
    xQueueSend(fileCommandes, &c, 0);
    serverWeb.send(200, "text/plain", "OK");
  });

  serverWeb.on("/close", []() {
    Serial.println("[WEB] --> Demande de FERMETURE reçue");
    CommandeToit c = CMD_FERMER;
    xQueueSend(fileCommandes, &c, 0);
    serverWeb.send(200, "text/plain", "OK");
  });

  serverWeb.on("/reset", []() {
    Serial.println("[WEB] --> Demande de RESET reçue");
    CommandeToit c = CMD_RESET;
    xQueueSend(fileCommandes, &c, 0);
    serverWeb.send(200, "text/plain", "OK");
  });

  // Interface Web OTA avec Barre de progression
  serverWeb.on("/update", HTTP_GET, []() {
    serverWeb.sendHeader("Connection", "close");
    String html = R"rawliteral(
    <!DOCTYPE html>
    <html>
    <head>
      <meta charset='UTF-8'>
      <meta name='viewport' content='width=device-width, initial-scale=1'>
      <style>
        body {background:#111; color:#eee; text-align:center; padding:30px; font-family:sans-serif;}
        form {background:#222; padding:30px; border-radius:10px; display:inline-block; border:1px solid #444; width:80%; max-width:400px;}
        input[type='file'] {margin:20px 0; font-size:16px; color:#eee;}
        .btn {padding:15px 30px; background:#2f855a; color:#fff; border:none; border-radius:8px; font-weight:bold; cursor:pointer; font-size:16px; width:100%;}
        .btn:hover {background:#276749;}
        #prg-container {display:none; margin-top:20px;}
        progress {width:100%; height:25px; border-radius:5px;}
        #status {margin-top:10px; font-weight:bold; color:#4fd1c5;}
      </style>
    </head>
    <body>
      <h2>Mise a jour Firmware (OTA)</h2>
      <form id="upload_form" onsubmit="event.preventDefault(); uploadFile();">
        <input type='file' id='file_input' name='update' accept='.bin' required><br>
        <input type='submit' id='upload_btn' class='btn' value='Flasher l ESP32'>
        <div id="prg-container">
          <progress id="prg" value="0" max="100"></progress>
          <div id="status">0%</div>
        </div>
      </form>
      <br><br><a href='/' style='color:#888; text-decoration:none;'>Retour accueil</a>
      
      <script>
        function uploadFile() {
          var file = document.getElementById("file_input").files[0];
          var formdata = new FormData();
          formdata.append("update", file);
          
          var ajax = new XMLHttpRequest();
          ajax.upload.addEventListener("progress", progressHandler, false);
          ajax.addEventListener("load", completeHandler, false);
          ajax.addEventListener("error", errorHandler, false);
          ajax.addEventListener("abort", abortHandler, false);
          
          ajax.open("POST", "/update");
          ajax.send(formdata);
          
          document.getElementById("prg-container").style.display = "block";
          document.getElementById("upload_btn").style.display = "none";
        }
        
        function progressHandler(event) {
          var percent = Math.round((event.loaded / event.total) * 100);
          document.getElementById("prg").value = percent;
          document.getElementById("status").innerHTML = percent + "% uploade... Patientez pour le flash.";
        }
        
        function completeHandler(event) {
          document.getElementById("status").innerHTML = event.target.responseText;
          document.getElementById("prg").value = 100;
          if (event.target.responseText.includes("ECHEC")) {
             document.getElementById("status").style.color = "#e53e3e";
          } else {
             document.getElementById("status").style.color = "#48bb78";
          }
        }
        
        function errorHandler(event) {
          document.getElementById("status").innerHTML = "Echec de connexion.";
          document.getElementById("status").style.color = "#e53e3e";
        }
        
        function abortHandler(event) {
          document.getElementById("status").innerHTML = "Upload annule.";
        }
      </script>
    </body>
    </html>
    )rawliteral";
    
    serverWeb.send(200, "text/html", html);
  });

  serverWeb.on("/update", HTTP_POST, []() {
    serverWeb.sendHeader("Connection", "close");
    serverWeb.send(200, "text/plain", (Update.hasError()) ? "ECHEC DE LA MISE A JOUR" : "SUCCES ! Redemarrage de l'ESP32 en cours...");
    ESP.restart();
  }, []() {
    HTTPUpload& upload = serverWeb.upload();
    if (upload.status == UPLOAD_FILE_START) {
      if (!Update.begin(UPDATE_SIZE_UNKNOWN)) Update.printError(Serial);
    } else if (upload.status == UPLOAD_FILE_WRITE) {
      if (Update.write(upload.buf, upload.currentSize) != upload.currentSize) Update.printError(Serial);
    } else if (upload.status == UPLOAD_FILE_END) {
      if (Update.end(true)) Serial.printf("OTA OK : %u octets\n", upload.totalSize);
    }
  });

  // ===================== LE PIÈGE POUR NINA (DEBUG ULTIME) =====================
  serverWeb.onNotFound([]() {
    Serial.print(">>> [ALPACA 404] NINA A DEMANDÉ UNE ROUTE INCONNUE : ");
    Serial.println(serverWeb.uri());
    serverWeb.send(200, "application/json", "{\"ErrorNumber\":1024,\"ErrorMessage\":\"Endpoint non implemente\"}");
  });

  serverWeb.begin();
  server.begin(); // Démarrage du serveur ASCOM TCP (8888)

  for (;;) {
    // --- 1. GESTION WIFI SILENCIEUSE ET ROBUSTE ---
    if (WiFi.status() != WL_CONNECTED) {
      Serial.println("[WIFI] Déconnecté ! Tentative de reconnexion...");
      WiFi.disconnect();
      vTaskDelay(1000 / portTICK_PERIOD_MS);
      WiFi.begin(ssid, password);
      
      // On attend sagement 5 secondes sans spammer
      for (int i = 0; i < 50; i++) {
        if (WiFi.status() == WL_CONNECTED) {
          Serial.println("[WIFI] Reconnecté avec succès !");
          break;
        }
        vTaskDelay(100 / portTICK_PERIOD_MS);
      }
      continue; // On repasse au début de la boucle
    }

    serverWeb.handleClient(); // Traitement Web, OTA et API Alpaca

    // --- 2. GESTION ANCIENNE ASCOM TCP (Port 8888) ---
    if (client) {
      if (client.connected()) {
        while (client.available()) {
          char c = client.read();
          if (c == '#') {
            ascom(String(networkIn));
            memset(networkIn, 0, sizeof(networkIn));
            netIndex = 0;
          } else if (netIndex < sizeof(networkIn) - 1) {
            networkIn[netIndex++] = c;
          }
        }
      } else {
        client.stop();
      }
    } else {
      client = server.available();
    }

    vTaskDelay(10 / portTICK_PERIOD_MS);
  }
}

// ===================== SETUP ===============================
void setup() {
  Serial.begin(115200); 
  
  pinMode(pinOpenRelay, OUTPUT);
  pinMode(pinCloseRelay, OUTPUT);

  digitalWrite(pinOpenRelay, LOW);
  digitalWrite(pinCloseRelay, LOW);

  pinMode(pinOpenedSensor, INPUT_PULLUP);
  pinMode(pinClosedSensor, INPUT_PULLUP);
  pinMode(pinSafeSensor, INPUT_PULLUP);

  // --- Pré-chargement pour éviter la fausse alerte du debounce ---
  capteurSafe.etatStable = digitalRead(pinSafeSensor);
  capteurSafe.dernier = capteurSafe.etatStable;

  capteurOuvert.etatStable = digitalRead(pinOpenedSensor);
  capteurOuvert.dernier = capteurOuvert.etatStable;
  
  capteurFerme.etatStable = digitalRead(pinClosedSensor);
  capteurFerme.dernier = capteurFerme.etatStable;

  // --- Lecture intiale protégée ---
  if (capteurSafe.etatStable == SAFE_BLOCKED) {
    etatCourant = ETAT_SAFE_BLOCKED;
  }
  else if (capteurOuvert.etatStable == LOW && capteurFerme.etatStable == LOW) {
    etatCourant = ETAT_ERREUR;
  }
  else if (capteurOuvert.etatStable == LOW) {
    etatCourant = ETAT_OUVERT;
  }
  else if (capteurFerme.etatStable == LOW) {
    etatCourant = ETAT_FERME;
  }
  else {
    etatCourant = ETAT_INCONNU;
  }

  fileCommandes = xQueueCreate(10, sizeof(CommandeToit));
  xTaskCreatePinnedToCore(codeNet, "NET", 10000, NULL, 1, NULL, 0);
  xTaskCreatePinnedToCore(codeSafe, "SAFE", 10000, NULL, 5, NULL, 1);
}

// ===================== LOOP ================================
EtatToit dernierEtatAffiche = ETAT_INCONNU;

void loop() {
//  vTaskDelete(NULL); // A décommenter une fois les tests réalisés sur breadboard

  // 1. Affichage en temps réel des changements d'état
  if (etatCourant != dernierEtatAffiche) {
    dernierEtatAffiche = etatCourant;
    Serial.print(">>> STATUT TOIT : ");
    switch(etatCourant) {
      case ETAT_FERME:          Serial.println("FERMÉ"); break;
      case ETAT_OUVERT:         Serial.println("OUVERT"); break;
      case ETAT_EN_OUVERTURE:   Serial.println("EN OUVERTURE..."); break;
      case ETAT_EN_FERMETURE:   Serial.println("EN FERMETURE..."); break;
      case ETAT_ERREUR:         Serial.println("ERREUR CAPTEUR / TIMEOUT !"); break;
      case ETAT_SAFE_BLOCKED:   Serial.println("SÉCURITÉ BLOQUÉE (Vérifier IPX800) !"); break;
      default:                  Serial.println("INCONNU"); break;
    }
  }

  // 2. Écoute des commandes tapées au clavier (Test sur bureau)
  if (Serial.available()) {
    char c = Serial.read();
    if (c == 'o' || c == 'O') {
      CommandeToit cmd = CMD_OUVRIR;
      xQueueSend(fileCommandes, &cmd, 0);
      Serial.println("--> ORDRE ENVOYÉ : OUVRIR");
    } 
    else if (c == 'f' || c == 'F') {
      CommandeToit cmd = CMD_FERMER;
      xQueueSend(fileCommandes, &cmd, 0);
      Serial.println("--> ORDRE ENVOYÉ : FERMER");
    } 
    else if (c == 'r' || c == 'R') {
      CommandeToit cmd = CMD_RESET;
      xQueueSend(fileCommandes, &cmd, 0);
      Serial.println("--> ORDRE ENVOYÉ : RESET");
    }
  }
  
  vTaskDelay(50 / portTICK_PERIOD_MS);
}
