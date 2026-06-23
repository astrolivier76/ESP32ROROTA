// ==========================================================
// Motorisation Observatoire ESP32 + Web OTA + ASCOM ALPACA
// Architecture Fail-Safe, Haute Performance & Conformité ASCOM
// ==========================================================

#include <WiFi.h>
#include <WebServer.h>
#include <ESPmDNS.h>
#include <esp_task_wdt.h>
#include <Update.h>
#include "config.h"

// ============= Variables de comptage requetes =============
volatile int compteurRequetes = 0;
unsigned long chronoRequetes = 0;

// ===================== RÉSEAU & ASCOM =====================
WebServer serverWeb(80);    // Web, OTA & Alpaca
volatile uint32_t serverTransactionID = 1; // Compteur obligatoire pour Alpaca

// ===================== ETATS ===============================
enum EtatToit {
  ETAT_INCONNU, ETAT_FERME, ETAT_OUVERT,
  ETAT_EN_OUVERTURE, ETAT_EN_FERMETURE,
  ETAT_ERREUR, ETAT_SAFE_BLOCKED
};

// Séparation claire entre ABORT (arrêt d'urgence) et RESET (acquittement)
enum CommandeToit { CMD_AUCUNE, CMD_OUVRIR, CMD_FERMER, CMD_ABORT, CMD_RESET };

volatile EtatToit etatCourant = ETAT_INCONNU;
QueueHandle_t fileCommandes;

unsigned long chronoMouvement = 0;
unsigned long timerRelais = 0;
bool isAlpacaConnected = false;

// ===================== CAPTEURS ============================
struct Capteur {
  int pin;
  bool etatStable;
  bool dernier;
  unsigned long t;
  unsigned long debounce;
};

// Utilisation des variables du config.h avec debounce 500ms
Capteur capteurOuvert  = {pinOpenedSensor, HIGH, HIGH, 0, 500}; 
Capteur capteurFerme   = {pinClosedSensor, HIGH, HIGH, 0, 500};
Capteur capteurSafe    = {pinSafeSensor, HIGH, HIGH, 0, 500};

#define SAFE_OK LOW
#define SAFE_BLOCKED HIGH

// ===================== LOGIQUE MATÉRIELLE =====================
void lire(Capteur &c) {
  bool v = digitalRead(c.pin);
  if (v != c.dernier) c.t = millis();
  if (millis() - c.t > c.debounce) c.etatStable = v;
  c.dernier = v;
}

void stopUrgence() {
  digitalWrite(pinOpenRelay, LOW);
  digitalWrite(pinCloseRelay, LOW);
}

void transition(EtatToit e) {
  if (e == ETAT_SAFE_BLOCKED || e == ETAT_ERREUR) { stopUrgence(); etatCourant = e; return; }
  if (etatCourant == ETAT_SAFE_BLOCKED) return;
  switch (etatCourant) {
    case ETAT_INCONNU: etatCourant = e; break;
    case ETAT_FERME: if (e == ETAT_EN_OUVERTURE) etatCourant = e; break;
    case ETAT_OUVERT: if (e == ETAT_EN_FERMETURE) etatCourant = e; break;
    case ETAT_EN_OUVERTURE: if (e == ETAT_OUVERT || e == ETAT_ERREUR) etatCourant = e; break;
    case ETAT_EN_FERMETURE: if (e == ETAT_FERME || e == ETAT_ERREUR) etatCourant = e; break;
    default: break;
  }
}

void majCapteurs() {
  if (etatCourant == ETAT_SAFE_BLOCKED && capteurSafe.etatStable == SAFE_OK) { etatCourant = ETAT_INCONNU; return; }
  if (capteurSafe.etatStable == SAFE_BLOCKED) {
    if (etatCourant != ETAT_OUVERT && etatCourant != ETAT_SAFE_BLOCKED) { stopUrgence(); etatCourant = ETAT_SAFE_BLOCKED; }
    return; 
  }
  if (capteurOuvert.etatStable == LOW && capteurFerme.etatStable == LOW) {
    if (etatCourant != ETAT_ERREUR) transition(ETAT_ERREUR); return;
  }
  if (capteurOuvert.etatStable == LOW) transition(ETAT_OUVERT);
  else if (capteurFerme.etatStable == LOW) transition(ETAT_FERME);
  if (etatCourant == ETAT_OUVERT && capteurOuvert.etatStable == HIGH) transition(ETAT_ERREUR);
  if (etatCourant == ETAT_FERME && capteurFerme.etatStable == HIGH) transition(ETAT_ERREUR);
}

void exec(CommandeToit c) {
  if (c == CMD_ABORT) {
    stopUrgence();
    // Si on était en mouvement, on est maintenant bloqué au milieu de nulle part
    if (etatCourant == ETAT_EN_OUVERTURE || etatCourant == ETAT_EN_FERMETURE) etatCourant = ETAT_INCONNU;
    return;
  }
  if (c == CMD_RESET) { 
    stopUrgence(); 
    etatCourant = ETAT_INCONNU; // Acquitte l'erreur
    return; 
  }
  
  if (etatCourant == ETAT_SAFE_BLOCKED || etatCourant == ETAT_ERREUR) return;

  if (c == CMD_OUVRIR && etatCourant == ETAT_FERME) {
    digitalWrite(pinCloseRelay, LOW); digitalWrite(pinOpenRelay, HIGH);
    chronoMouvement = millis(); timerRelais = millis(); transition(ETAT_EN_OUVERTURE);
  }
  if (c == CMD_FERMER && etatCourant == ETAT_OUVERT && capteurSafe.etatStable == SAFE_OK) {
    digitalWrite(pinOpenRelay, LOW); digitalWrite(pinCloseRelay, HIGH);
    chronoMouvement = millis(); timerRelais = millis(); transition(ETAT_EN_FERMETURE);
  }
}

void checkTimeout() {
  if ((etatCourant == ETAT_EN_OUVERTURE || etatCourant == ETAT_EN_FERMETURE) && millis() - chronoMouvement > TIMEOUT_MOUVEMENT) {
    stopUrgence(); etatCourant = ETAT_ERREUR;
  }
}

void majRelais() {
  if (millis() - timerRelais > 1000) { digitalWrite(pinOpenRelay, LOW); digitalWrite(pinCloseRelay, LOW); }
}

// ===================== TACHE SECURITE ======================
void codeSafe(void *p) {
  esp_task_wdt_config_t twdt_config = { .timeout_ms = WDT_TIMEOUT * 1000, .idle_core_mask = (1 << portNUM_PROCESSORS) - 1, .trigger_panic = true, };
  esp_task_wdt_init(&twdt_config); esp_task_wdt_add(NULL);
  CommandeToit c;
  for (;;) {
    lire(capteurOuvert); lire(capteurFerme); lire(capteurSafe);
    majCapteurs();
    if (xQueueReceive(fileCommandes, &c, 0) == pdPASS) exec(c);
    majRelais(); checkTimeout();
    esp_task_wdt_reset(); vTaskDelay(10 / portTICK_PERIOD_MS);
  }
}

// ===================== MOTEUR ALPACA ULTRA-LÉGER =====================
void sendAlpaca(const char* value, int errNum = 0, const char* errMsg = "") {
  //comptage requetes
  compteurRequetes++;
  //fin de comptage
  char buf[300];
  unsigned long cid = serverWeb.hasArg("ClientTransactionID") ? serverWeb.arg("ClientTransactionID").toInt() : 0;
  uint32_t sid = serverTransactionID++; // Incrémentation propre à ASCOM
  
  if (value != nullptr) {
    snprintf(buf, sizeof(buf), "{\"Value\":%s,\"ClientTransactionID\":%lu,\"ServerTransactionID\":%lu,\"ErrorNumber\":%d,\"ErrorMessage\":\"%s\"}", value, cid, sid, errNum, errMsg);
  } else {
    snprintf(buf, sizeof(buf), "{\"ClientTransactionID\":%lu,\"ServerTransactionID\":%lu,\"ErrorNumber\":%d,\"ErrorMessage\":\"%s\"}", cid, sid, errNum, errMsg);
  }
  
  serverWeb.sendHeader("Connection", "close"); 
  serverWeb.send(200, "application/json", buf);
}

// ===================== TACHE NET & WEB =====================
void codeNet(void *p) {
  WiFi.setSleep(false);
  WiFi.begin(ssid, password);
  while (WiFi.status() != WL_CONNECTED) { vTaskDelay(500 / portTICK_PERIOD_MS); Serial.print("."); }
  Serial.println("\nConnecté ! IP : " + WiFi.localIP().toString());

  if (MDNS.begin(mdns_name)) MDNS.addService("http", "tcp", 80);

  // --- API LOCALE & PAGE WEB ---
  serverWeb.on("/api/status", HTTP_GET, []() {
    const char* e = "INCONNU";
    if(etatCourant==ETAT_FERME) e="FERMÉ"; else if(etatCourant==ETAT_OUVERT) e="OUVERT"; else if(etatCourant==ETAT_EN_OUVERTURE) e="EN OUVERTURE..."; else if(etatCourant==ETAT_EN_FERMETURE) e="EN FERMETURE..."; else if(etatCourant==ETAT_ERREUR) e="ERREUR CAPTEUR / TIMEOUT"; else if(etatCourant==ETAT_SAFE_BLOCKED) e="SÉCURITÉ TOIT BLOQUÉE";
    serverWeb.sendHeader("Connection", "close"); 
    serverWeb.send(200, "text/plain", e);
  });

  // --- ASCOM ALPACA : MANAGEMENT ---
  serverWeb.on("/management/apiversions", HTTP_GET, []() { sendAlpaca("[1]"); });
  serverWeb.on("/management/v1/configureddevices", HTTP_GET, []() { sendAlpaca("[{\"DeviceName\":\"ESP32 Observatoire\",\"DeviceType\":\"Dome\",\"DeviceNumber\":0,\"UniqueID\":\"ESP32-TOIT-123456\"}]"); });
  serverWeb.on("/management/v1/description", HTTP_GET, []() { sendAlpaca("{\"ServerName\":\"ESP32\",\"Manufacturer\":\"ToitAstro\",\"ManufacturerVersion\":\"1.0\",\"Location\":\"Observatoire\"}"); });

  // --- ASCOM ALPACA : BASE API ---
  serverWeb.on("/api/v1/dome/0/name", HTTP_GET, []() { sendAlpaca("\"ESP32 Observatoire\""); });
  serverWeb.on("/api/v1/dome/0/description", HTTP_GET, []() { sendAlpaca("\"Toit ROR ESP32\""); });
  serverWeb.on("/api/v1/dome/0/driverinfo", HTTP_GET, []() { sendAlpaca("\"ESP32 Alpaca v1.0\""); });
  serverWeb.on("/api/v1/dome/0/driverversion", HTTP_GET, []() { sendAlpaca("\"1.0\""); });
  serverWeb.on("/api/v1/dome/0/interfaceversion", HTTP_GET, []() { sendAlpaca("1"); });
  serverWeb.on("/api/v1/dome/0/supportedactions", HTTP_GET, []() { sendAlpaca("[]"); });
  
  // NOUVEAU : Propriété Connecting commune à l'API Alpaca
  serverWeb.on("/api/v1/dome/0/connecting", HTTP_GET, []() { sendAlpaca("false"); });

  // --- ASCOM ALPACA : DOME PROPRIÉTÉS ---
  serverWeb.on("/api/v1/dome/0/connected", HTTP_GET, []() { sendAlpaca(isAlpacaConnected ? "true" : "false"); });
  serverWeb.on("/api/v1/dome/0/connected", HTTP_PUT, []() { 
    if(serverWeb.hasArg("Connected")) isAlpacaConnected = serverWeb.arg("Connected").equalsIgnoreCase("True"); 
    sendAlpaca(nullptr); 
  });
  
  serverWeb.on("/api/v1/dome/0/shutterstatus", HTTP_GET, []() {
    const char* s = "4"; 
    if(etatCourant==ETAT_OUVERT) s="0"; else if(etatCourant==ETAT_FERME) s="1"; else if(etatCourant==ETAT_EN_OUVERTURE) s="2"; else if(etatCourant==ETAT_EN_FERMETURE) s="3";
    sendAlpaca(s);
  });
  serverWeb.on("/api/v1/dome/0/slewing", HTTP_GET, []() { sendAlpaca((etatCourant==ETAT_EN_OUVERTURE||etatCourant==ETAT_EN_FERMETURE) ? "true" : "false"); });
  serverWeb.on("/api/v1/dome/0/atpark", HTTP_GET, []() { sendAlpaca((etatCourant==ETAT_FERME) ? "true" : "false"); });
  serverWeb.on("/api/v1/dome/0/athome", HTTP_GET, []() { sendAlpaca("false"); });
  serverWeb.on("/api/v1/dome/0/slaved", HTTP_GET, []() { sendAlpaca("false"); });
  serverWeb.on("/api/v1/dome/0/altitude", HTTP_GET, []() { sendAlpaca("0"); });
  serverWeb.on("/api/v1/dome/0/azimuth", HTTP_GET, []() { sendAlpaca("0"); });

  serverWeb.on("/api/v1/dome/0/canfindhome", HTTP_GET, []() { sendAlpaca("false"); });
  serverWeb.on("/api/v1/dome/0/canpark", HTTP_GET, []() { sendAlpaca("true"); }); // Autorisé pour fermer le toit
  serverWeb.on("/api/v1/dome/0/cansetshutter", HTTP_GET, []() { sendAlpaca("true"); });
  serverWeb.on("/api/v1/dome/0/cansetaltitude", HTTP_GET, []() { sendAlpaca("false"); });
  serverWeb.on("/api/v1/dome/0/cansetazimuth", HTTP_GET, []() { sendAlpaca("false"); });
  serverWeb.on("/api/v1/dome/0/canslave", HTTP_GET, []() { sendAlpaca("false"); });
  serverWeb.on("/api/v1/dome/0/cansyncazimuth", HTTP_GET, []() { sendAlpaca("false"); });
  serverWeb.on("/api/v1/dome/0/cansetpark", HTTP_GET, []() { sendAlpaca("false"); });

  // --- ASCOM ALPACA : DOME ACTIONS ---
  serverWeb.on("/api/v1/dome/0/openshutter", HTTP_PUT, []() { 
    CommandeToit c = CMD_OUVRIR; 
    if(xQueueSend(fileCommandes, &c, 0) == pdPASS) sendAlpaca(nullptr); 
    else sendAlpaca(nullptr, 1024, "File d'attente pleine");
  });
  serverWeb.on("/api/v1/dome/0/closeshutter", HTTP_PUT, []() { 
    CommandeToit c = CMD_FERMER; 
    if(xQueueSend(fileCommandes, &c, 0) == pdPASS) sendAlpaca(nullptr); 
    else sendAlpaca(nullptr, 1024, "File d'attente pleine");
  });
  serverWeb.on("/api/v1/dome/0/park", HTTP_PUT, []() { 
    CommandeToit c = CMD_FERMER; // Mapper Park à la fermeture
    if(xQueueSend(fileCommandes, &c, 0) == pdPASS) sendAlpaca(nullptr); 
    else sendAlpaca(nullptr, 1024, "File d'attente pleine");
  });
  serverWeb.on("/api/v1/dome/0/abortslew", HTTP_PUT, []() { 
    CommandeToit c = CMD_ABORT; 
    if(xQueueSend(fileCommandes, &c, 0) == pdPASS) sendAlpaca(nullptr); 
    else sendAlpaca(nullptr, 1024, "File d'attente pleine");
  });

  serverWeb.on("/api/v1/dome/0/slaved", HTTP_PUT, []() { sendAlpaca(nullptr, 1024, "Non applicable"); });
  serverWeb.on("/api/v1/dome/0/setpark", HTTP_PUT, []() { sendAlpaca(nullptr, 1024, "Non applicable"); });
  serverWeb.on("/api/v1/dome/0/findhome", HTTP_PUT, []() { sendAlpaca(nullptr, 1024, "Non applicable"); });
  serverWeb.on("/api/v1/dome/0/slewtoazimuth", HTTP_PUT, []() { sendAlpaca(nullptr, 1024, "Non applicable"); });
  serverWeb.on("/api/v1/dome/0/syncazimuth", HTTP_PUT, []() { sendAlpaca(nullptr, 1024, "Non applicable"); });
  serverWeb.on("/api/v1/dome/0/slewtoaltitude", HTTP_PUT, []() { sendAlpaca(nullptr, 1024, "Non applicable"); });

  // --- ACTIONS WEB MANUELLES ---
  serverWeb.on("/open", []() { 
    CommandeToit c = CMD_OUVRIR; 
    serverWeb.sendHeader("Connection", "close"); 
    if(xQueueSend(fileCommandes, &c, 0) == pdPASS) serverWeb.send(200, "text/plain", "OK"); 
    else serverWeb.send(503, "text/plain", "BUSY");
  });
  serverWeb.on("/close", []() { 
    CommandeToit c = CMD_FERMER; 
    serverWeb.sendHeader("Connection", "close"); 
    if(xQueueSend(fileCommandes, &c, 0) == pdPASS) serverWeb.send(200, "text/plain", "OK");
    else serverWeb.send(503, "text/plain", "BUSY");
  });
  serverWeb.on("/abort", []() { 
    CommandeToit c = CMD_ABORT; 
    serverWeb.sendHeader("Connection", "close"); 
    if(xQueueSend(fileCommandes, &c, 0) == pdPASS) serverWeb.send(200, "text/plain", "OK");
    else serverWeb.send(503, "text/plain", "BUSY");
  });
  serverWeb.on("/reset", []() { 
    CommandeToit c = CMD_RESET; 
    serverWeb.sendHeader("Connection", "close"); 
    if(xQueueSend(fileCommandes, &c, 0) == pdPASS) serverWeb.send(200, "text/plain", "OK");
    else serverWeb.send(503, "text/plain", "BUSY");
  });

  // --- PAGE D'ACCUEIL ---
  serverWeb.on("/", HTTP_GET, []() {
    serverWeb.sendHeader("Connection", "close");
    serverWeb.send(200, "text/html", R"rawliteral(<!DOCTYPE html><html><head><meta charset='UTF-8'><meta name='viewport' content='width=device-width, initial-scale=1'><style>body{background:#111;color:#eee;text-align:center;padding:30px;font-family:sans-serif;}.card{background:#222;padding:30px;border-radius:10px;display:inline-block;border:1px solid #444;width:80%;max-width:500px;}.btn{padding:12px 24px;margin:10px;color:#fff;border:none;border-radius:6px;font-weight:bold;cursor:pointer;font-size:16px;text-decoration:none;display:inline-block;}.btn-open{background:#2f855a;}.btn-open:hover{background:#276749;}.btn-close{background:#c53030;}.btn-close:hover{background:#9b2c2c;}.btn-abort{background:#e53e3e;}.btn-abort:hover{background:#c53030;}.btn-reset{background:#dd6b20;}.btn-reset:hover{background:#c05621;}.btn-ota{background:#4a5568;}.btn-ota:hover{background:#2d3748;}#badge-etat{padding:8px 15px;border-radius:20px;font-weight:bold;background:#4a5568;display:inline-block;margin-top:15px;}.moving{background:#dd6b20 !important;color:#fff;}.open{background:#38a169 !important;color:#fff;}.close{background:#e53e3e !important;color:#fff;}</style></head><body><div class="card"><h2>Contrôle de l'Abri Astro</h2><hr style="border-color:#444;"><p>État actuel du toit :</p><div id="badge-etat">CHARGEMENT...</div><br><br><button onclick="envoyerCommande('/open')" class="btn btn-open">Ouvrir</button><button onclick="envoyerCommande('/close')" class="btn btn-close">Fermer</button><button onclick="envoyerCommande('/abort')" class="btn btn-abort">Stop</button><br><button onclick="envoyerCommande('/reset')" class="btn btn-reset">Acquitter Erreur</button><br><br><a href="/update" class="btn btn-ota" style="font-size:12px;">Mise à jour Firmware (OTA)</a></div><script>function rafraichirEtat(){var xhr=new XMLHttpRequest();xhr.open("GET","/api/status",true);xhr.onload=function(){if(xhr.status===200){var etat=xhr.responseText;var badge=document.getElementById("badge-etat");badge.innerHTML=etat;badge.className="";if(etat.includes("OUVERT")&&!etat.includes("EN"))badge.classList.add("open");else if(etat.includes("FERMÉ")&&!etat.includes("EN"))badge.classList.add("close");else if(etat.includes("EN"))badge.classList.add("moving");else badge.classList.add("moving");}};xhr.send();}function envoyerCommande(url){var xhr=new XMLHttpRequest();xhr.open("GET",url,true);xhr.send();setTimeout(rafraichirEtat,200);}rafraichirEtat();setInterval(rafraichirEtat,500);</script></body></html>)rawliteral");
  });

  // --- MISE A JOUR OTA ---
  serverWeb.on("/update", HTTP_GET, []() {
    serverWeb.sendHeader("Connection", "close");
    serverWeb.send(200, "text/html", R"rawliteral(<!DOCTYPE html><html><head><meta charset='UTF-8'><meta name='viewport' content='width=device-width, initial-scale=1'><style>body{background:#111;color:#eee;text-align:center;padding:30px;font-family:sans-serif;}form{background:#222;padding:30px;border-radius:10px;display:inline-block;border:1px solid #444;width:80%;max-width:400px;}input[type='file']{margin:20px 0;font-size:16px;color:#eee;}.btn{padding:15px 30px;background:#2f855a;color:#fff;border:none;border-radius:8px;font-weight:bold;cursor:pointer;font-size:16px;width:100%;}.btn:hover{background:#276749;}#prg-container{display:none;margin-top:20px;}progress{width:100%;height:25px;border-radius:5px;}#status{margin-top:10px;font-weight:bold;color:#4fd1c5;}</style></head><body><h2>Mise a jour Firmware (OTA)</h2><form id="upload_form" onsubmit="event.preventDefault(); uploadFile();"><input type='file' id='file_input' name='update' accept='.bin' required><br><input type='submit' id='upload_btn' class='btn' value='Flasher l ESP32'><div id="prg-container"><progress id="prg" value="0" max="100"></progress><div id="status">0%</div></div></form><br><br><a href='/' style='color:#888;text-decoration:none;'>Retour accueil</a><script>function uploadFile(){var file=document.getElementById("file_input").files[0];var formdata=new FormData();formdata.append("update",file);var ajax=new XMLHttpRequest();ajax.upload.addEventListener("progress",progressHandler,false);ajax.addEventListener("load",completeHandler,false);ajax.addEventListener("error",errorHandler,false);ajax.addEventListener("abort",abortHandler,false);ajax.open("POST","/update");ajax.send(formdata);document.getElementById("prg-container").style.display="block";document.getElementById("upload_btn").style.display="none";}function progressHandler(event){var percent=Math.round((event.loaded/event.total)*100);document.getElementById("prg").value=percent;document.getElementById("status").innerHTML=percent+"% uploade... Patientez pour le flash.";}function completeHandler(event){document.getElementById("status").innerHTML=event.target.responseText;document.getElementById("prg").value=100;if(event.target.responseText.includes("ECHEC")){document.getElementById("status").style.color="#e53e3e";}else{document.getElementById("status").style.color="#48bb78";}}function errorHandler(event){document.getElementById("status").innerHTML="Echec de connexion.";document.getElementById("status").style.color="#e53e3e";}function abortHandler(event){document.getElementById("status").innerHTML="Upload annule.";}</script></body></html>)rawliteral");
  });

  serverWeb.on("/update", HTTP_POST, []() {
    serverWeb.sendHeader("Connection", "close");
    serverWeb.send(200, "text/plain", (Update.hasError()) ? "ECHEC DE LA MISE A JOUR" : "SUCCES ! Redemarrage de l'ESP32 en cours...");
    ESP.restart();
  }, []() {
    HTTPUpload& upload = serverWeb.upload();
    if (upload.status == UPLOAD_FILE_START) { if (!Update.begin(UPDATE_SIZE_UNKNOWN)) Update.printError(Serial); } 
    else if (upload.status == UPLOAD_FILE_WRITE) { if (Update.write(upload.buf, upload.currentSize) != upload.currentSize) Update.printError(Serial); } 
    else if (upload.status == UPLOAD_FILE_END) { if (Update.end(true)) Serial.printf("OTA OK : %u octets\n", upload.totalSize); }
  });

  serverWeb.onNotFound([]() { sendAlpaca(nullptr, 1024, "Endpoint non implemente"); });

  serverWeb.begin();

  for (;;) {
    if (WiFi.status() != WL_CONNECTED) {
      WiFi.disconnect(); vTaskDelay(1000 / portTICK_PERIOD_MS); WiFi.begin(ssid, password);
      for (int i = 0; i < 50; i++) { if (WiFi.status() == WL_CONNECTED) break; vTaskDelay(100 / portTICK_PERIOD_MS); }
      continue;
    }
    
    serverWeb.handleClient(); // Traitement Web instantané
    vTaskDelay(1 / portTICK_PERIOD_MS); 
  }
}

// ===================== SETUP & LOOP ========================
void setup() {
  Serial.begin(115200); 
  pinMode(pinOpenRelay, OUTPUT); pinMode(pinCloseRelay, OUTPUT);
  digitalWrite(pinOpenRelay, LOW); digitalWrite(pinCloseRelay, LOW);

  pinMode(pinOpenedSensor, INPUT_PULLUP); pinMode(pinClosedSensor, INPUT_PULLUP); pinMode(pinSafeSensor, INPUT_PULLUP);

  capteurSafe.etatStable = digitalRead(pinSafeSensor); capteurSafe.dernier = capteurSafe.etatStable;
  capteurOuvert.etatStable = digitalRead(pinOpenedSensor); capteurOuvert.dernier = capteurOuvert.etatStable;
  capteurFerme.etatStable = digitalRead(pinClosedSensor); capteurFerme.dernier = capteurFerme.etatStable;

  if (capteurSafe.etatStable == SAFE_BLOCKED) etatCourant = ETAT_SAFE_BLOCKED;
  else if (capteurOuvert.etatStable == LOW && capteurFerme.etatStable == LOW) etatCourant = ETAT_ERREUR;
  else if (capteurOuvert.etatStable == LOW) etatCourant = ETAT_OUVERT;
  else if (capteurFerme.etatStable == LOW) etatCourant = ETAT_FERME;
  else etatCourant = ETAT_INCONNU;

  fileCommandes = xQueueCreate(10, sizeof(CommandeToit));
  
  // Le serveur Web va sur le Core 1 (Application)
  xTaskCreatePinnedToCore(codeNet, "NET", 10000, NULL, 1, NULL, 1); 
  // La boucle de sécurité (très légère) va sur le Core 0 avec la gestion Wi-Fi native
  xTaskCreatePinnedToCore(codeSafe, "SAFE", 10000, NULL, 5, NULL, 0); 
}

EtatToit dernierEtatAffiche = ETAT_INCONNU;
void loop() {
  if (millis() - chronoRequetes >= 1000) {
    if (compteurRequetes > 0) {
      Serial.print("[CHARGE RÉSEAU] Requêtes Alpaca par seconde : ");
      Serial.println(compteurRequetes);
      compteurRequetes = 0; // On remet à zéro pour la seconde suivante
    }
    chronoRequetes = millis();
  }
  vTaskDelay(50 / portTICK_PERIOD_MS);
}
