// ==========================================================
// Motorisation Observatoire ESP32 + RRCI ASCOM 1.3.1 + WEB OTA
// Architecture Fail-Safe matérielle via IPX800
// ==========================================================

#include <WiFi.h>
#include <WebServer.h>
#include <ESPmDNS.h>      // Ajout de la bibliothèque mDNS
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
  // 1. PRIORITÉ ABSOLUE : SÉCURITÉ IPX800
  if (capteurSafe.etatStable == SAFE_BLOCKED) {
    if (etatCourant != ETAT_SAFE_BLOCKED) {
      stopUrgence();
      etatCourant = ETAT_SAFE_BLOCKED;
    }
    return;
  }

  // 2. PARADOXE MATÉRIEL : Les deux capteurs fin de course sont à LOW
  if (capteurOuvert.etatStable == LOW && capteurFerme.etatStable == LOW) {
    if (etatCourant != ETAT_ERREUR) {
      transition(ETAT_ERREUR); 
    }
    return;
  }

  // 3. LECTURE NORMALE : Mise à jour de l'état
  if (capteurOuvert.etatStable == LOW) transition(ETAT_OUVERT);
  else if (capteurFerme.etatStable == LOW) transition(ETAT_FERME);

  // 4. PERTE DE CAPTEUR ANORMALE (Toit physiquement poussé ou capteur arraché)
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
  WiFi.begin(ssid, password);

  // --- NOUVEAU BLOC : On attend la connexion et on affiche l'IP ---
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

  // Page d'accueil principale dynamique avec AJAX
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
        <!-- NOUVEAUX BOUTONS AJAX -->
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

        // Envoi des commandes en arrière-plan (AJAX)
        function envoyerCommande(url) {
          var xhr = new XMLHttpRequest();
          xhr.open("GET", url, true);
          xhr.send();
          // Force une mise à jour immédiate du badge 200ms après le clic
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

  // Actions Boutons (AJAX - Pas de rechargement de page)
  serverWeb.on("/open", []() {
    CommandeToit c = CMD_OUVRIR;
    xQueueSend(fileCommandes, &c, 0);
    serverWeb.send(200, "text/plain", "OK");
  });

  serverWeb.on("/close", []() {
    CommandeToit c = CMD_FERMER;
    xQueueSend(fileCommandes, &c, 0);
    serverWeb.send(200, "text/plain", "OK");
  });

  serverWeb.on("/reset", []() {
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
  Serial.begin(115200); // Ajout conseillé pour le débogage (OTA et mDNS)
  
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
EtatToit dernierEtatAffiche = ETAT_INCONNU; // A supprimer lors de la compilation finale sans loop()

void loop() {
//  vTaskDelete(NULL); a decommmenter une fois les tests realisés sur breadboard

// bloc de test sur breadboard

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

  // 2. Écoute des commandes tapées au clavier
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
// fin d'instertion bloc de test esp32
  
  vTaskDelay(50 / portTICK_PERIOD_MS); // Petite pause pour ne pas saturer le processeur
}
