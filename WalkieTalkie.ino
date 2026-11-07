#include <esp_now.h>
#include <esp_wifi.h>
#include <WiFi.h>
#include "SSD1306Wire.h"
#include <math.h>
#include <string.h>

// ======================================================
// OLED
// ======================================================
SSD1306Wire display(0x3C, 5, 4);

// ======================================================
// CONFIGURACAO ESP-NOW (IGUAL A PLACA A)
// ======================================================

// MAC da PLACA A (a outra placa). Confira no Serial da placa A.
uint8_t MAC_PAR[6] = {0x84, 0xCC, 0xA8, 0x5D, 0x0C, 0x9C};

#define REDE_ID            0x42     // identifica pacotes do nosso projeto
#define CANAL_ESPNOW       1        // precisa ser o mesmo nas duas placas
#define INTERVALO_PING_MS  1000
#define TIMEOUT_LINK_MS    3500     // sem PONG por esse tempo = link caiu

#define TIPO_PING 1
#define TIPO_PONG 2
#define TIPO_PTT  3   // estado do botao PTT (campo id: 1 = apertado, 0 = solto)

// Mesma struct da placa A (8 bytes). Se mudar aqui, mude la tambem.
typedef struct __attribute__((packed)) {
  uint8_t  rede;        // REDE_ID
  uint8_t  tipo;        // TIPO_PING ou TIPO_PONG
  uint16_t id;          // numero de sequencia
  uint32_t t_envio_us;  // micros() de quem mandou o PING
} struct_mensagem;

// ---- Mensagem de texto (digitada no Serial Monitor) ----
#define TIPO_TXT        4        // tipo de pacote de texto
#define TXT_MAX         40       // maximo de caracteres por mensagem
#define MSG_EXIBIR_MS   10000    // tempo que a mensagem recebida fica na tela
#define LARG_MSG_PX     126      // largura util (em pixels) para o texto na tela

typedef struct __attribute__((packed)) {
  uint8_t  rede;                 // REDE_ID
  uint8_t  tipo;                 // TIPO_TXT
  uint16_t id;                   // numero de sequencia
  char     texto[TXT_MAX + 1];   // texto terminado em '\0'
} struct_texto;

// Variaveis alteradas nos callbacks (outra tarefa) -> volatile
volatile uint32_t rttUs          = 0;
volatile uint32_t ultimoPongMs   = 0;
volatile uint32_t pongsRecebidos = 0;
volatile uint32_t falhasEnvio    = 0;
volatile int      rssiPar        = -100;

volatile bool     pongPendente   = false;
volatile uint16_t pongId         = 0;
volatile uint32_t pongT          = 0;

// Botao PTT da outra placa (atualizado no callback)
volatile bool     pttRemotoRaw      = false;
volatile uint32_t ultimoPttRemotoMs = 0;

// Mensagem de texto recebida (callback grava, loop consome)
char          msgRx[TXT_MAX + 1];
volatile bool msgRxPendente = false;

uint32_t pingsEnviados = 0;
bool     espnowOk      = false;
bool     linkOk        = false;
int      rssiAtual     = 0;

// ======================================================
// CALLBACK DE ENVIO (assinatura mudou no core 3.3)
// ======================================================
#if ESP_ARDUINO_VERSION >= ESP_ARDUINO_VERSION_VAL(3, 3, 0)
void aoEnviar(const wifi_tx_info_t *info, esp_now_send_status_t status) {
#else
void aoEnviar(const uint8_t *mac, esp_now_send_status_t status) {
#endif
  if (status != ESP_NOW_SEND_SUCCESS) falhasEnvio++;
}

// ======================================================
// CALLBACK DE RECEBIMENTO
// ======================================================
void aoReceber(const esp_now_recv_info_t *info, const uint8_t *dados, int len) {
  // Mensagem de texto (tem tamanho proprio)
  if (len == sizeof(struct_texto)) {
    struct_texto t;
    memcpy(&t, dados, sizeof(t));
    if (t.rede == REDE_ID && t.tipo == TIPO_TXT && !msgRxPendente) {
      t.texto[TXT_MAX] = '\0';
      strcpy(msgRx, t.texto);
      msgRxPendente = true;                        // o loop processa
    }
    return;
  }

  if (len != sizeof(struct_mensagem)) return;      // tamanho errado: ignora

  struct_mensagem m;
  memcpy(&m, dados, sizeof(m));
  if (m.rede != REDE_ID) return;                   // nao e do nosso projeto

  rssiPar = info->rx_ctrl->rssi;                   // forca do sinal do par

  if (m.tipo == TIPO_PING) {
    // Nao envia aqui dentro: so marca para o loop responder
    pongId = m.id;
    pongT  = m.t_envio_us;
    pongPendente = true;
  } else if (m.tipo == TIPO_PONG) {
    rttUs = micros() - m.t_envio_us;               // ida e volta
    ultimoPongMs = millis();
    pongsRecebidos++;
  } else if (m.tipo == TIPO_PTT) {
    pttRemotoRaw = (m.id != 0);                    // par apertou/soltou o botao
    ultimoPttRemotoMs = millis();
  }
}

// ======================================================
// INICIAR ESP-NOW COM CANAL FIXO
// ======================================================
void iniciarEspNow() {
  WiFi.mode(WIFI_STA);                             // radio em modo STA
  WiFi.disconnect();                               // sem conectar em rede

  esp_wifi_set_channel(CANAL_ESPNOW, WIFI_SECOND_CHAN_NONE);

  uint8_t canal; wifi_second_chan_t sec;
  esp_wifi_get_channel(&canal, &sec);
  Serial.printf("[WIFI] Canal atual: %d\n", canal);

  if (esp_now_init() != ESP_OK) {
    Serial.println("[ESP-NOW] ERRO ao iniciar");
    return;
  }
  esp_now_register_send_cb(aoEnviar);
  esp_now_register_recv_cb(aoReceber);

  esp_now_peer_info_t peer = {};
  memcpy(peer.peer_addr, MAC_PAR, 6);
  peer.channel = 0;                                // 0 = canal atual
  peer.encrypt = false;
  peer.ifidx   = WIFI_IF_STA;

  if (esp_now_add_peer(&peer) != ESP_OK) {
    Serial.println("[ESP-NOW] ERRO ao adicionar o par");
    return;
  }
  espnowOk = true;
  Serial.println("[ESP-NOW] Iniciado");
}

// ======================================================
// PING-PONG (chamar sempre no loop)
// ======================================================
void gerenciarEspNow() {
  static uint32_t ultimoPing = 0;
  static uint16_t idPing = 0;
  static uint32_t pongsImpressos = 0;
  uint32_t agora = millis();

  if (!espnowOk) { linkOk = false; return; }

  // 1) Responde PING recebido
  if (pongPendente) {
    pongPendente = false;
    struct_mensagem r = { REDE_ID, TIPO_PONG, pongId, pongT };
    esp_now_send(MAC_PAR, (uint8_t *)&r, sizeof(r));
  }

  // 2) Envia PING periodico
  if (agora - ultimoPing >= INTERVALO_PING_MS) {
    ultimoPing = agora;
    struct_mensagem p = { REDE_ID, TIPO_PING, ++idPing, micros() };
    esp_now_send(MAC_PAR, (uint8_t *)&p, sizeof(p));
    pingsEnviados++;
  }

  // 3) Estado do link
  linkOk = (pongsRecebidos > 0) && (agora - ultimoPongMs < TIMEOUT_LINK_MS);
  if (linkOk) rssiAtual = rssiPar;

  // 4) Mostra no Serial cada PONG novo
  if (pongsRecebidos != pongsImpressos) {
    pongsImpressos = pongsRecebidos;
    Serial.printf("[PONG] RTT=%.2f ms | RSSI=%d dBm | enviados=%lu recebidos=%lu falhas=%lu\n",
                  rttUs / 1000.0, rssiPar,
                  (unsigned long)pingsEnviados,
                  (unsigned long)pongsRecebidos,
                  (unsigned long)falhasEnvio);
  }

  // 5) Sem link: mostra o status a cada 2 s para diagnostico
  static uint32_t ultimoStatus = 0;
  if (!linkOk && agora - ultimoStatus >= 2000) {
    ultimoStatus = agora;
    Serial.printf("[STATUS] sem link | enviados=%lu recebidos=%lu falhas=%lu\n",
                  (unsigned long)pingsEnviados,
                  (unsigned long)pongsRecebidos,
                  (unsigned long)falhasEnvio);
  }
}

// ======================================================
// PINOS
// ======================================================
#define PINO_BOTAO   21
#define PINO_BATERIA 33

// true = imprime o estado do botao no Serial a cada 300 ms (polui o chat)
#define DEBUG_BOTAO  false

// ======================================================
// DIVISOR DE TENSAO
// ======================================================
const float R1 = 10000.0;
const float R2 = 10000.0;
const float FATOR_CALIBRACAO = 1.000;

// ======================================================
// INTERVALOS
// ======================================================
const unsigned long INTERVALO_ADC  = 500;
const unsigned long INTERVALO_OLED = 50;
const unsigned long INTERVALO_TESTE_BOTAO = 300;

// ======================================================
// VARIAVEIS
// ======================================================
float tensaoBateria = 0.0;
float nivelAudio = 0.0;

int leituraBotao = HIGH;

unsigned long ultimaLeituraADC = 0;
unsigned long ultimaAtualizacaoOLED = 0;
unsigned long ultimaLeituraTeste = 0;

bool estadoBotao = false;

// ======================================================
// LEITURA DA TENSAO DA BATERIA
// ======================================================
float lerTensaoBateria() {
  const int AMOSTRAS = 20;
  uint32_t somaMilliVolts = 0;

  for (int i = 0; i < AMOSTRAS; i++) {
    somaMilliVolts += analogReadMilliVolts(PINO_BATERIA);
  }

  float tensaoPino = (somaMilliVolts / (float)AMOSTRAS) / 1000.0;
  float fatorDivisor = (R1 + R2) / R2;

  return tensaoPino * fatorDivisor * FATOR_CALIBRACAO;
}

// ======================================================
// LEITURA DO BOTAO (GPIO 21 -> BOTAO -> GND, LOW = PRESSIONADO)
// ======================================================
void lerBotao() {
  static int      ultimaBruta = HIGH;
  static uint32_t tMudou = 0;

  int bruta = digitalRead(PINO_BOTAO);
  if (bruta != ultimaBruta) { ultimaBruta = bruta; tMudou = millis(); }
  if (millis() - tMudou >= 25) {                 // debounce de 25 ms
    leituraBotao = bruta;
    estadoBotao = (bruta == LOW);
  }

#if DEBUG_BOTAO
  if (millis() - ultimaLeituraTeste >= INTERVALO_TESTE_BOTAO) {
    ultimaLeituraTeste = millis();

    Serial.print("GPIO 21 = ");
    Serial.print(leituraBotao);
    Serial.print(" | PTT = ");
    Serial.println(estadoBotao ? "FALANDO" : "LIVRE");
  }
#endif
}

// ======================================================
// PTT SINCRONIZADO COM A OUTRA PLACA
// ======================================================
#define INTERVALO_PTT_MS 300    // reenvia o estado do botao periodicamente
#define TIMEOUT_PTT_MS   1000   // sem aviso do par por esse tempo = considera solto

#define PTT_LIVRE      0
#define PTT_FALANDO    1
#define PTT_ESCUTANDO  2
#define PTT_CONFLITO   3

bool pttRemoto = false;         // true = a outra placa esta com o botao apertado

void gerenciarPtt() {
  static bool     ultimoEnviado = false;
  static uint32_t ultimoEnvioMs = 0;
  uint32_t agora = millis();

  if (!espnowOk) { pttRemoto = false; return; }

  // Envia na hora quando o botao muda; fora isso, reenvia a cada INTERVALO_PTT_MS
  if (estadoBotao != ultimoEnviado || agora - ultimoEnvioMs >= INTERVALO_PTT_MS) {
    ultimoEnviado = estadoBotao;
    ultimoEnvioMs = agora;
    struct_mensagem m = { REDE_ID, TIPO_PTT, (uint16_t)(estadoBotao ? 1 : 0), micros() };
    esp_now_send(MAC_PAR, (uint8_t *)&m, sizeof(m));
  }

  // Estado do botao do par (expira se ele parar de avisar)
  pttRemoto = pttRemotoRaw && (agora - ultimoPttRemotoMs < TIMEOUT_PTT_MS);
}

int estadoPtt() {
  if (estadoBotao && pttRemoto) return PTT_CONFLITO;
  if (estadoBotao)              return PTT_FALANDO;
  if (pttRemoto)                return PTT_ESCUTANDO;
  return PTT_LIVRE;
}

// ======================================================
// MENSAGENS (Serial -> ESP-NOW -> tela)
// ======================================================
char     msgTx[TXT_MAX + 1] = "";     // mensagem digitada, aguardando o botao
bool     msgTxPendente = false;

char     msgExibida[TXT_MAX + 1] = "";  // ultima mensagem recebida
String   msgL1 = "", msgL2 = "";        // ela quebrada em 2 linhas
uint32_t msgExibidaMs = 0;

bool msgAtiva() {
  return msgExibida[0] != '\0' && (millis() - msgExibidaMs < MSG_EXIBIR_MS);
}

// Quebra o texto em ate 2 linhas que cabem em maxPx pixels (o excesso e cortado)
void quebrarMensagem(const char *txt, String &l1, String &l2, int maxPx) {
  String resto = String(txt);

  int corte = resto.length();
  while (corte > 0 && display.getStringWidth(resto.substring(0, corte)) > maxPx) corte--;
  if (corte < (int)resto.length()) {              // nao coube tudo: tenta quebrar no espaco
    int esp = resto.lastIndexOf(' ', corte);
    if (esp > 0) corte = esp;
  }
  l1 = resto.substring(0, corte);

  resto = resto.substring(corte);
  resto.trim();
  int c2 = resto.length();
  while (c2 > 0 && display.getStringWidth(resto.substring(0, c2)) > maxPx) c2--;
  l2 = resto.substring(0, c2);
}

// Le o Serial: cada linha (Enter) vira uma mensagem pendente
void lerSerial() {
  static char buf[TXT_MAX + 1];
  static int  n = 0;
  static bool truncou = false;

  while (Serial.available()) {
    char c = Serial.read();
    if (c == '\r') continue;
    if (c == '\n') {
      if (n > 0) {
        buf[n] = '\0';
        strcpy(msgTx, buf);
        msgTxPendente = true;
        Serial.printf("[MSG] pronta: \"%s\"%s -> segure o botao para enviar\n",
                      msgTx, truncou ? " (cortada em 40 caracteres)" : "");
      }
      n = 0;
      truncou = false;
    } else if (n < TXT_MAX) {
      buf[n++] = (c >= 32 && c <= 126) ? c : '?';   // so ASCII (a fonte do display nao tem acentos)
    } else {
      truncou = true;
    }
  }
}

void gerenciarMensagens() {
  static uint16_t idTxt = 0;
  static bool avisouConflito = false;
  static bool avisouSemLink  = false;

  // 1) Mensagem recebida: descarta se eu tambem estou apertando (conflito)
  if (msgRxPendente) {
    int est = estadoPtt();
    if (est == PTT_FALANDO || est == PTT_CONFLITO) {
      Serial.printf("[MSG] descartada (conflito): \"%s\"\n", msgRx);
    } else {
      strcpy(msgExibida, msgRx);
      msgExibidaMs = millis();
      display.setFont(ArialMT_Plain_10);
      quebrarMensagem(msgExibida, msgL1, msgL2, LARG_MSG_PX);
      Serial.printf("[MSG RX] %s\n", msgExibida);
    }
    msgRxPendente = false;
  }

  // 2) Envio: so com o botao apertado, sem conflito e com link
  int est = estadoPtt();
  if (est != PTT_FALANDO) avisouSemLink = false;
  if (est != PTT_CONFLITO) avisouConflito = false;

  if (msgTxPendente) {
    if (est == PTT_FALANDO) {
      if (linkOk) {
        struct_texto t = {};
        t.rede = REDE_ID;
        t.tipo = TIPO_TXT;
        t.id   = ++idTxt;
        strncpy(t.texto, msgTx, TXT_MAX);
        esp_now_send(MAC_PAR, (uint8_t *)&t, sizeof(t));
        Serial.printf("[MSG TX] %s\n", msgTx);
        msgTxPendente = false;
      } else if (!avisouSemLink) {
        avisouSemLink = true;
        Serial.println("[MSG] sem link: mensagem retida ate o link voltar");
      }
    } else if (est == PTT_CONFLITO && !avisouConflito) {
      avisouConflito = true;
      Serial.println("[MSG] CONFLITO: mensagem retida ate o outro soltar o botao");
    }
  }
}

// ======================================================
// SIMULACAO DO NIVEL DE AUDIO
// ======================================================
float simularNivelAudio() {
  float tempo = millis() / 1000.0;
  float modulacao = (sin(tempo * 1.7) + 1.0) / 2.0;
  return 0.15 + modulacao * 0.85;
}

// ======================================================
// DESENHAR ONDA SONORA SIMULADA
// ======================================================
void desenharOndaSenoidal(float amplitude) {
  const int centroY = 49;
  const int amplitudeMaxima = 12;
  const float frequencia = 0.18;

  float tempo = millis() / 1000.0;

  for (int x = 0; x < 128; x += 6) {
    display.setPixel(x, centroY);
  }

  for (int x = 0; x < 128; x++) {
    float fase = x * frequencia - tempo * 5.0;
    int y = centroY - (int)(sin(fase) * amplitude * amplitudeMaxima);

    if (y < 34) y = 34;
    if (y > 62) y = 62;

    display.setPixel(x, y);
    if (y + 1 <= 62) display.setPixel(x, y + 1);
  }
}

// ======================================================
// ATUALIZAR OLED
// ======================================================
void atualizarDisplay() {
  display.clear();
  display.setFont(ArialMT_Plain_10);

  // MENSAGEM RECEBIDA (area livre no topo)
  if (msgAtiva()) {
    display.drawString(0, 0,  msgL1);
    display.drawString(0, 12, msgL2);
  }

  // BATERIA

  display.drawHorizontalLine(0, 25, 128);
  display.drawString(0, 27, "Bateria: " + String(tensaoBateria, 2) + " V");

  // PTT (estado sincronizado com a outra placa)
  int est = estadoPtt();
  if (est == PTT_CONFLITO) {
    if ((millis() / 300) % 2) {            // pisca em video invertido
      display.fillRect(0, 39, 128, 12);
      display.setColor(BLACK);
    }
    display.drawString(0, 39, "!! CONFLITO !!");
    display.setColor(WHITE);
  } else if (est == PTT_FALANDO) {
    display.drawString(0, 39, "PTT: FALANDO");
    desenharOndaSenoidal(nivelAudio);
  } else if (est == PTT_ESCUTANDO) {
    display.drawString(0, 39, "PTT: ESCUTANDO");
  } else {
    display.drawString(0, 39, "PTT: LIVRE");
  }

  // STATUS DO RADIO
  display.drawString(0, 54, linkOk ? "RADIO: OK" : "RADIO: SEM LINK");

  display.display();
}

// ======================================================
// SETUP
// ======================================================
void setup() {
  Serial.begin(115200);
  delay(200);

  Serial.println();
  Serial.println("INICIALIZANDO SISTEMA...");

  pinMode(PINO_BOTAO, INPUT_PULLUP);

  analogReadResolution(12);
  analogSetPinAttenuation(PINO_BATERIA, ADC_11db);

  display.init();
  display.clear();
  display.setFont(ArialMT_Plain_10);
  display.drawString(0, 20, "Iniciando sistema...");
  display.display();

  tensaoBateria = lerTensaoBateria();
  Serial.printf("Tensao da bateria: %.2f V\n", tensaoBateria);

  iniciarEspNow();

  // Mostra o MAC desta placa (precisa bater com o MAC_PAR da placa A)
  Serial.print("MAC LOCAL: ");
  Serial.println(WiFi.macAddress());

  Serial.println("SISTEMA PRONTO");
  Serial.println("[MSG] Digite a mensagem no Serial Monitor (Nova linha) e segure o botao para enviar");
}

// ======================================================
// LOOP
// ======================================================
void loop() {
  unsigned long agora = millis();

  gerenciarEspNow();
  lerBotao();
  gerenciarPtt();
  lerSerial();
  gerenciarMensagens();

  nivelAudio = estadoBotao ? simularNivelAudio() : 0.0;

  if (agora - ultimaLeituraADC >= INTERVALO_ADC) {
    ultimaLeituraADC = agora;
    tensaoBateria = lerTensaoBateria();
  }

  if (agora - ultimaAtualizacaoOLED >= INTERVALO_OLED) {
    ultimaAtualizacaoOLED = agora;
    atualizarDisplay();
  }
}