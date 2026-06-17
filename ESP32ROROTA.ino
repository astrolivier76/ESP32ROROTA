// ==========================================================
// Motorisation Observatoire ESP32 + RRCI ASCOM 1.3.1 + WEB OTA
// Architecture Fail-Safe matérielle via IPX800
// ==========================================================

#include <WiFi.h>
#include <WebServer.h>
#include <esp_task_wdt.h>
#include <Update.h>
#include "config.h"

// ===================== RÉSEAU =============================
WiFiServer server(portTCP); // Port ASCOM (ex: 8888)
WebServer serverWeb(80);    // Port HTTP Interface & OTA
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
  // SAFE LOGICIEL (La coupure matérielle est gérée en amont par l'IPX800)
  if (capteurSafe.etatStable == SAFE_BLOCKED) {
    if (etatCourant != ETAT_SAFE_BLOCKED) {
      stopUrgence();
      etatCourant = ETAT_SAFE_BLOCKED;
    }
    return;
  }

  // Fins de course (Arrivée)
  if (capteurOuvert.etatStable == LOW) transition(ETAT_OUVERT);
  if (capteurFerme.etatStable == LOW) transition(ETAT_FERME);

  // DÉTECTION DE PERTE DE POSITION (La parade "Auto-Reverse")
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
  // Impulsion d'une seconde pour l'optocoupleur PC817
  if (millis() - timerRelais > 1000) {
    digitalWrite(pinOpenRelay, LOW);
    digitalWrite(pinCloseRelay, LOW);
  }
}

// ===================== ASCOM (RRCI V1.3.1) =================
void ascom(String cmd) {
  cmd.trim();
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
  else if (cmd.startsWith("setsafe") || cmd.startsWith("setmotion")) {
    client.print("OK:");
    client.print(cmd);
    client.print("#");
  }
  else if (cmd == "getpulsecount") {
    client.print("PULSES:0#");
  }
  else if (cmd == "resetpulse") {
    client.print("OK:resetpulse#");
  }
  else {
    client.print("ERR:");
    client.print(cmd);
    client.print("#");
  }
}

// ===================== TACHE SECURITE (Core 1) =============
void codeSafe(void *p) {
  esp_task_wdt_init(WDT_TIMEOUT, true);
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

    esp_task_wdt_reset();
    vTaskDelay(10 / portTICK_PERIOD_MS);
  }
}

// ===================== TACHE NET & WEB (Core 0) ============
void codeNet(void *p) {
  WiFi.config(local_IP, gateway, subnet, primaryDNS);
  WiFi.begin(ssid, password);

  // Interface Web Accueil
  serverWeb.on("/", []() {
    String html = "<!DOCTYPE html><html><head><meta charset='UTF-8'><meta name='viewport' content='width=device-width, initial-scale=1'><style>body{background:#111;color:#eee;text-align:center;padding:20px;font-family:sans-serif;}.btn{display:inline-block;padding:15px 30px;margin:10px;background:#2b6cb0;color:#fff;text-decoration:none;border-radius:8px;font-weight:bold;border:none;cursor:pointer;}.btn-red{background:#c53030;}.btn-green{background:#2f855a;}</style></head><body><h1>Observatoire</h1>";
    html += "<p>Etat actuel gere par ASCOM</p><hr style='border:1px solid #333;margin:20px 0;'>";
    html += "<button class='btn' onclick=\"fetch('/open')\">Ouvrir Toit</button>";
    html += "<button class='btn btn-red' onclick=\"fetch('/close')\">Fermer Toit</button><br>";
    html += "<button class='btn btn-green' onclick=\"fetch('/reset')\">Acquitter Erreur (Reset)</button>";
    html += "<hr style='border:1px solid #333;margin:20px 0;'><a href='/update' class='btn' style='background:#555;'>Mise a jour Firmware (OTA)</a></body></html>";
    serverWeb.send(200, "text/html", html);
  });

  serverWeb.on("/open", []() {
    serverWeb.sendHeader("Access-Control-Allow-Origin", "*");
    CommandeToit c = CMD_OUVRIR;
    xQueueSend(fileCommandes, &c, 0);
    serverWeb.send(200, "text/plain", "Ouverture demandee");
  });

  serverWeb.on("/close", []() {
    serverWeb.sendHeader("Access-Control-Allow-Origin", "*");
    CommandeToit c = CMD_FERMER;
    xQueueSend(fileCommandes, &c, 0);
    serverWeb.send(200, "text/plain", "Fermeture demandee");
  });

  serverWeb.on("/reset", []() {
    serverWeb.sendHeader("Access-Control-Allow-Origin", "*");
    CommandeToit c = CMD_RESET;
    xQueueSend(fileCommandes, &c, 0);
    serverWeb.send(200, "text/plain", "Securite re-armee");
  });

  // Interface Web OTA
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
          
          // Afficher la barre de progression et cacher le bouton pour éviter le double clic
          document.getElementById("prg-container").style.display = "block";
          document.getElementById("upload_btn").style.display = "none";
        }
        
        function progressHandler(event) {
          var percent = Math.round((event.loaded / event.total) * 100);
          document.getElementById("prg").value = percent;
          document.getElementById("status").innerHTML = percent + "% uploade... Patientez pour le flash.";
        }
        
        function completeHandler(event) {
          // Affiche le message de succès ou d'erreur généré par l'ESP32 (HTTP_POST handler)
          document.getElementById("status").innerHTML = event.target.responseText;
          document.getElementById("prg").value = 100;
          if (event.target.responseText.includes("ECHEC")) {
             document.getElementById("status").style.color = "#e53e3e"; // Rouge si échec
          } else {
             document.getElementById("status").style.color = "#48bb78"; // Vert si succès
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

  serverWeb.on("/state", []() {
    serverWeb.sendHeader("Access-Control-Allow-Origin", "*"); 
    
    String reponse = "INCONNU";
    if (etatCourant == ETAT_OUVERT) reponse = "OUVERT";
    else if (etatCourant == ETAT_FERME) reponse = "FERMÉ";
    else if (etatCourant == ETAT_EN_OUVERTURE) reponse = "EN OUVERTURE...";
    else if (etatCourant == ETAT_EN_FERMETURE) reponse = "EN FERMETURE...";
    else if (etatCourant == ETAT_SAFE_BLOCKED) reponse = "BLOCAGE SÉCURITÉ (Monture non parquée)";
    else if (etatCourant == ETAT_ERREUR) reponse = "ERREUR (Défaut fin de course)";

    serverWeb.send(200, "text/plain", reponse);
  });

  serverWeb.begin();
  server.begin(); // Démarrage du serveur ASCOM (8888)

  for (;;) {
    if (WiFi.status() != WL_CONNECTED) {
      WiFi.disconnect();
      WiFi.begin(ssid, password);
      vTaskDelay(2000 / portTICK_PERIOD_MS);
      continue;
    }

    serverWeb.handleClient();

    if (!client || !client.connected()) client = server.available();

    if (client && client.available()) {
      char c = client.read();
      if (c == '#') {
        ascom(String(networkIn));
        memset(networkIn, 0, sizeof(networkIn));
        netIndex = 0;
      } else if (netIndex < sizeof(networkIn) - 1) {
        networkIn[netIndex++] = c;
      }
    }
    vTaskDelay(10 / portTICK_PERIOD_MS);
  }
}

// ===================== SETUP ===============================
void setup() {
  pinMode(pinOpenRelay, OUTPUT);
  pinMode(pinCloseRelay, OUTPUT);

  digitalWrite(pinOpenRelay, LOW);
  digitalWrite(pinCloseRelay, LOW);

  pinMode(pinOpenedSensor, INPUT_PULLUP);
  pinMode(pinClosedSensor, INPUT_PULLUP);
  pinMode(pinSafeSensor, INPUT_PULLUP);

  if (digitalRead(pinOpenedSensor) == LOW) etatCourant = ETAT_OUVERT;
  else if (digitalRead(pinClosedSensor) == LOW) etatCourant = ETAT_FERME;
  else etatCourant = ETAT_INCONNU;

  fileCommandes = xQueueCreate(10, sizeof(CommandeToit));

  xTaskCreatePinnedToCore(codeNet, "NET", 10000, NULL, 1, NULL, 0);
  xTaskCreatePinnedToCore(codeSafe, "SAFE", 10000, NULL, 5, NULL, 1);
}

// ===================== LOOP ================================
void loop() {
  vTaskDelete(NULL);
}
