#include <SafeFall_Voice_Recognition_inferencing.h>
#include <Wire.h>
#include <Adafruit_GFX.h>
#include <Adafruit_SSD1306.h>
#include <Adafruit_MPU6050.h>
#include <Adafruit_Sensor.h>
#include <driver/i2s_std.h>
#include <driver/gpio.h>
#include <math.h>

// CONFIGURAÇÕES BLYNK ONLINE
#define USAR_BLYNK 1

#if USAR_BLYNK
  #define BLYNK_PRINT Serial
  #define BLYNK_TEMPLATE_ID "TMPL5csEqHYIe"
  #define BLYNK_TEMPLATE_NAME "SafeFall Pulseira"
  #define BLYNK_AUTH_TOKEN "4UxZ2I8PSaYD3cj1N_qAr0kZKBuHVRwJ"

  #include <WiFi.h>
  #include <BlynkSimpleEsp32.h>

  char ssid[] = "sturi";
  char pass[] = "Pedrovieira";

  bool blynkConfigurado = false;
  unsigned long ultimaTentativaBlynk = 0;
  unsigned long ultimoEnvioBlynk = 0;
#endif

// EDGE IMPULSE — "SOCORRO"
#include <SafeFall_Voice_Recognition_inferencing.h>
#define SAFE_FALL_TEM_EDGE_IMPULSE 1

// PINOS — ESP32-WROOM-32E
#define I2C_SDA 21
#define I2C_SCL 22

#define BUZZER_PIN 14
#define BOTAO_PIN 27

#define I2S_SCK 26
#define I2S_WS 25
#define I2S_SD 33
#define I2S_PORT I2S_NUM_0

#define BATT_ADC_PIN 34

// OLED
#define SCREEN_WIDTH 128
#define SCREEN_HEIGHT 64
#define OLED_RESET -1
#define OLED_ADDR 0x3C

Adafruit_SSD1306 display(SCREEN_WIDTH, SCREEN_HEIGHT, &Wire, OLED_RESET);
Adafruit_MPU6050 mpu;

// ÍCONE WIFI
static const unsigned char PROGMEM icon_wifi[] = {
  0x3F, 0x80,
  0xC0, 0x60,
  0x00, 0x00,
  0x1F, 0x00,
  0x31, 0x80,
  0x00, 0x00,
  0x0E, 0x00,
  0x04, 0x00,
  0x04, 0x00
};

// ESTADOS
enum EstadoSistema {
  ESTADO_NORMAL,
  ESTADO_VALIDAR_QUEDA,
  ESTADO_PRE_ALERTA,
  ESTADO_ALERTA,
  ESTADO_CANCELADO
};

EstadoSistema estado = ESTADO_NORMAL;

// MÓDULOS
bool oledOK = false;
bool mpuOK = false;
bool micOK = false;

// MPU
float ax = 0, ay = 0, az = 0;
float gx = 0, gy = 0, gz = 0;

float gForceAtual = 1.0f;
float gyroDpsAtual = 0.0f;
float rollAtual = 0.0f;
float pitchAtual = 0.0f;
float temperaturaAtual = 0.0f;

float gRepouso = 1.0f;
float ruidoG = 0.03f;
float limiarImpactoG = 3.00f;
float limiarQuedaLivreG = 0.55f;
float toleranciaImobilidadeG = 0.14f;
float limiarImobilidadeGyroDps = 22.0f;
float rollReferencia = 0.0f;
float pitchReferencia = 0.0f;

const unsigned long PERIODO_IMU_MS = 10;
unsigned long ultimaAmostraIMUMs = 0;

const float ROTACAO_FORTE_DPS = 260.0f;
const float MUDANCA_ORIENTACAO_MIN_GRAUS = 60.0f;

const unsigned long TEMPO_QUEDA_LIVRE_MIN_MS = 120;
const unsigned long TEMPO_ROTACAO_FORTE_MIN_MS = 180;
const unsigned long JANELA_VALIDACAO_QUEDA_MS = 4000;
const unsigned long TEMPO_IMOBILIDADE_MS = 1500;

unsigned long inicioBaixaAceleracaoMs = 0;
unsigned long inicioRotacaoForteMs = 0;
unsigned long ultimaQuedaLivreMs = 0;
unsigned long ultimaRotacaoForteMs = 0;

unsigned long inicioValidacaoQuedaMs = 0;
unsigned long inicioJanelaQuedaDiretaMs = 0;
unsigned long inicioParadoAposImpactoMs = 0;
bool janelaQuedaDiretaAtiva = false;
float picoQuedaDiretaG = 0.0f;

float rollAnteriorRapido = 0.0f;
float pitchAnteriorRapido = 0.0f;
bool orientacaoRapidaInicializada = false;

// ALTERAÇÃO: IMPACTO_DIRETO_MIN_G 1.75 → 1.55 (mais sensível a quedas suaves/de costas)
const float IMPACTO_DIRETO_MIN_G = 1.55f;
const float GIRO_DIRETO_MIN_DPS = 150.0f;
const float MUDANCA_RAPIDA_MIN_GRAUS = 25.0f;
// ALTERAÇÃO: JANELA_QUEDA_DIRETA_MS 2500 → 3500 (mais tempo para validar queda lenta)
const unsigned long JANELA_QUEDA_DIRETA_MS = 3500;
// ALTERAÇÃO: TEMPO_PARADO_POS_IMPACTO_MS 500 → 700 (mais tempo para confirmar paragem)
const unsigned long TEMPO_PARADO_POS_IMPACTO_MS = 700;
unsigned long ultimoImpactoSemContextoMs = 0;
bool aguardarParagemAposImpacto = false;
float picoImpactoSemContextoG = 0.0f;

const unsigned long TEMPO_CONFIRMAR_PARAGEM_POS_IMPACTO_MS = 700;
const float IMPACTO_DIRETO_EXTRA_G = 0.15f;
unsigned long imovelDesdeMs = 0;

bool eventoHouveQuedaLivre = false;
bool eventoHouveRotacao = false;
float eventoPicoImpactoG = 0.0f;
float eventoPicoGyroDps = 0.0f;
float eventoMaiorMudancaOrientacao = 0.0f;
float eventoRollAntes = 0.0f;
float eventoPitchAntes = 0.0f;

// ALERTA
const unsigned long TEMPO_PRE_ALERTA_MS = 15000;
const unsigned long TEMPO_CANCELAR_BOTAO_MS = 2000;

bool alertaEnviado = false;
unsigned long inicioPreAlertaMs = 0;
String origemAlerta = "";
String ultimaDetecao = "Nenhuma";

unsigned long ultimoBuzzerMs = 0;
unsigned long ultimoDisplayMs = 0;

// BOTÃO
bool botaoAnterior = HIGH;
bool botaoEstavel = HIGH;
bool ultimaLeituraBotao = HIGH;
unsigned long ultimaMudancaBotaoMs = 0;
unsigned long inicioPressaoBotaoMs = 0;
bool cancelamentoConfirmado = false;

const unsigned long DEBOUNCE_BOTAO_MS = 50;

// DISPLAY
int paginaAtual = 0;
const int TOTAL_PAGINAS = 2;

// MICROFONE / VOZ
int micNivelMedio = 0;
int micPico = 0;
float probabilidadeSocorro = 0.0f;

unsigned long bloquearVozAteMs = 0;

// Handle do driver I2S
i2s_chan_handle_t i2sMicHandle = NULL;

#if SAFE_FALL_TEM_EDGE_IMPULSE
static bool debug_nn = false;
// ALTERAÇÃO: PROB_SOCORRO_FORTE 0.82 → 0.75 (disparo direto mais acessível)
const float PROB_SOCORRO_FORTE = 0.65f;
// ALTERAÇÃO: PROB_SOCORRO_VOTO 0.62 → 0.55 (votos mais acessíveis)
const float PROB_SOCORRO_VOTO = 0.45f;
// ALTERAÇÃO: MARGEM_VOZ 0.22 → 0.15 (menos exigente na distinção de classes)
const float MARGEM_VOZ = 0.15f;
// ALTERAÇÃO: MIC_PICO_MIN_SOCORRO 1800 → 1200 (aceita voz mais suave ou afastada)
const int MIC_PICO_MIN_SOCORRO = 1200;
// ALTERAÇÃO: VOTOS_SOCORRO_NECESSARIOS 3 → 2 (menos confirmações necessárias)
const uint8_t VOTOS_SOCORRO_NECESSARIOS = 1;
// ALTERAÇÃO: JANELA_VOTOS_SOCORRO_MS 4500 → 6000 (mais tempo para acumular votos)
const unsigned long JANELA_VOTOS_SOCORRO_MS = 6000;
#endif

// BATERIA
float voltagemBateria = 0.0f;
int percentagemBateria = 0;
int rawADC = 0;

const float BATT_FATOR_DIVISOR = 2.0f;
const float BATT_VREF = 3.3f;
const float BATT_CALIBRACAO = 1.09f;

// PROTÓTIPOS
void amostrarIMU();
void cancelarAlerta(const String& motivo);

#if SAFE_FALL_TEM_EDGE_IMPULSE
int get_signal_data(size_t offset, size_t length, float* out_ptr);
#endif

// BLYNK ONLINE NÃO BLOQUEANTE
#if USAR_BLYNK

void iniciarBlynkOnline() {
  if (blynkConfigurado) return;

  blynkConfigurado = true;

  WiFi.mode(WIFI_STA);
  WiFi.setSleep(false);
  WiFi.setAutoReconnect(true);
  WiFi.persistent(false);

  // Potência moderada para reduzir picos de corrente.
  WiFi.setTxPower(WIFI_POWER_11dBm);

  Blynk.config(BLYNK_AUTH_TOKEN);

  Serial.println("[WiFi] A iniciar...");
  WiFi.begin(ssid, pass);
}

void gerirBlynkOnline() {
  if (!blynkConfigurado) return;

  unsigned long agora = millis();

  if (WiFi.status() != WL_CONNECTED) {
    return;
  }

  if (!Blynk.connected()) {
    if (agora - ultimaTentativaBlynk >= 10000) {
      ultimaTentativaBlynk = agora;
      Serial.println("[Blynk] A tentar ligar online...");
      Blynk.connect(1000);
    }
    return;
  }

  Blynk.run();
}

void enviarDadosBlynk() {
  if (!Blynk.connected()) return;
  if (millis() - ultimoEnvioBlynk < 3000) return;

  ultimoEnvioBlynk = millis();

  Blynk.virtualWrite(V3, temperaturaAtual);
  Blynk.virtualWrite(V4, (int)(probabilidadeSocorro * 100));
  Blynk.virtualWrite(V5, gForceAtual);
  Blynk.virtualWrite(V6, percentagemBateria);
}

BLYNK_WRITE(V2) {
  if (param.asInt() == 1) {
    if (estado == ESTADO_PRE_ALERTA || estado == ESTADO_ALERTA || estado == ESTADO_VALIDAR_QUEDA) {
      cancelarAlerta("Reset Blynk");
    }
  }
}

#endif

// BUZZER
void buzzerOff() {
  noTone(BUZZER_PIN);
}

void bipCurto() {
  tone(BUZZER_PIN, 2000);
  delay(120);
  noTone(BUZZER_PIN);
}

void bipDuplo() {
  for (int i = 0; i < 2; i++) {
    tone(BUZZER_PIN, 1800);
    delay(130);
    noTone(BUZZER_PIN);
    delay(120);
  }
}

void atualizarBuzzerAlerta() {
  unsigned long agora = millis();

  if (estado == ESTADO_PRE_ALERTA) {
    if (agora - ultimoBuzzerMs >= 450) {
      ultimoBuzzerMs = agora;
      static bool ligado = false;
      ligado = !ligado;

      if (ligado) tone(BUZZER_PIN, 2200);
      else noTone(BUZZER_PIN);
    }
  }

  else if (estado == ESTADO_ALERTA) {
    if (agora - ultimoBuzzerMs >= 120) {
      ultimoBuzzerMs = agora;
      int freq = 2600 + (int)(sinf(agora / 80.0f) * 700.0f);
      tone(BUZZER_PIN, freq);
    }
  }

  else {
    buzzerOff();
  }
}

// I2C / OLED
void limparBarramentoI2C() {
  pinMode(I2C_SDA, INPUT_PULLUP);
  pinMode(I2C_SCL, INPUT_PULLUP);
  delay(50);

  if (digitalRead(I2C_SDA) == LOW) {
    pinMode(I2C_SCL, OUTPUT);

    for (int i = 0; i < 9; i++) {
      digitalWrite(I2C_SCL, LOW);
      delayMicroseconds(10);
      digitalWrite(I2C_SCL, HIGH);
      delayMicroseconds(10);
    }
  }

  pinMode(I2C_SDA, INPUT_PULLUP);
  pinMode(I2C_SCL, INPUT_PULLUP);
  delay(50);
}

void iniciarOLED() {
  oledOK = false;

  for (int tentativa = 1; tentativa <= 3; tentativa++) {
    Serial.print("A iniciar OLED, tentativa ");
    Serial.println(tentativa);

    if (display.begin(SSD1306_SWITCHCAPVCC, OLED_ADDR)) {
      oledOK = true;

      display.clearDisplay();
      display.setTextColor(SSD1306_WHITE);
      display.setTextSize(1);
      display.setCursor(0, 0);
      display.display();

      Serial.println("OLED OK");
      delay(500);
      return;
    }

    Serial.println("OLED falhou");
    delay(500);
  }

  Serial.println("ERRO: OLED nao iniciou");
}

String nomeEstado() {
  switch (estado) {
    case ESTADO_NORMAL: return "NORMAL";
    case ESTADO_VALIDAR_QUEDA: return "A VALIDAR";
    case ESTADO_PRE_ALERTA: return "PRE-ALERTA";
    case ESTADO_ALERTA: return "ALERTA";
    case ESTADO_CANCELADO: return "CANCELADO";
  }

  return "DESCONHECIDO";
}

void desenharCabecalho() {
  display.setTextSize(1);

#if USAR_BLYNK
  if (WiFi.status() == WL_CONNECTED) {
    display.drawBitmap(0, 0, icon_wifi, 11, 10, SSD1306_WHITE);
  } else {
    display.setCursor(0, 0);
    display.print("?");
  }
#else
  display.setCursor(0, 0);
  display.print("?");
#endif

  display.setCursor(25, 0);
  display.print("SAFE-FALL");

  String txtBat = String(percentagemBateria) + "%";
  int posXBat = 108 - txtBat.length() * 6;
  display.setCursor(posXBat, 0);
  display.print(txtBat);

  display.drawRect(110, 0, 18, 9, SSD1306_WHITE);
  display.fillRect(128, 2, 2, 5, SSD1306_WHITE);

  int bw = map(constrain(percentagemBateria, 0, 100), 0, 100, 0, 14);
  display.fillRect(112, 2, bw, 5, SSD1306_WHITE);

  display.drawFastHLine(0, 11, 128, SSD1306_WHITE);
}

void atualizarOLED() {
  if (!oledOK) return;
  if (millis() - ultimoDisplayMs < 250) return;
  ultimoDisplayMs = millis();
  display.clearDisplay();
  display.setTextColor(SSD1306_WHITE);
  desenharCabecalho();
  bool botaoPremido = digitalRead(BOTAO_PIN) == LOW;
  if (botaoPremido && (estado == ESTADO_PRE_ALERTA || estado == ESTADO_ALERTA || estado == ESTADO_VALIDAR_QUEDA)) {
    unsigned long t = millis() - inicioPressaoBotaoMs;
    unsigned long restante = t >= TEMPO_CANCELAR_BOTAO_MS ? 0 : TEMPO_CANCELAR_BOTAO_MS - t;
    display.setTextSize(1);
    display.setCursor(14, 20);
    display.print("PRIME PARA CANCELAR");
    display.setTextSize(2);
    display.setCursor(38, 38);
    display.print(restante / 1000.0f, 1);
    display.print("s");
  }
  else if (paginaAtual == 0) {
    display.setTextSize(2);
    if (estado == ESTADO_PRE_ALERTA) {
      bool eSocorro = origemAlerta.indexOf("SOCORRO") >= 0;
      display.setCursor(4, 16);
      display.print("PRE-ALERTA");
      unsigned long decorrido = millis() - inicioPreAlertaMs;
      unsigned long faltam = decorrido >= TEMPO_PRE_ALERTA_MS ? 0 : TEMPO_PRE_ALERTA_MS - decorrido;
      display.setTextSize(1);
      display.setCursor(8, 32);
      display.print(eSocorro ? "Alerta por socorro" : "Alerta por queda");
      display.setCursor(8, 42);
      display.print("Envia em ");
      display.print((faltam + 999) / 1000);
      display.print("s");
      display.setCursor(5, 54);
      display.print("Cancelar: botao 2s");
    }
    else if (estado == ESTADO_ALERTA) {
      bool eSocorro = origemAlerta.indexOf("SOCORRO") >= 0;
      display.setCursor(18, 18);
      display.print("ALERTA!");
      display.setTextSize(1);
      display.setCursor(4, 43);
      display.print(eSocorro ? "Alerta por socorro" : "Alerta por queda");
      display.setCursor(5, 54);
      display.print("Cancelar: botao 2s");
    }
    else if (estado == ESTADO_VALIDAR_QUEDA) {
      display.setCursor(10, 18);
      display.print("VALIDAR");
      display.setTextSize(1);
      display.setCursor(0, 43);
      display.print("A confirmar queda real");
      display.setCursor(0, 54);
      display.print("Mov. brusco sera ignorado");
    }
    else if (estado == ESTADO_CANCELADO) {
      display.setCursor(6, 18);
      display.print("CANCEL.");
      display.setTextSize(1);
      display.setCursor(0, 43);
      display.print("Alerta cancelado");
      display.setCursor(0, 54);
      display.print("Solte o botao");
    }
    else {
      display.setCursor(28, 18);
      display.print("SEGURO");
      display.setTextSize(1);
      display.setCursor(0, 42);
      display.print("Status: ");
      display.print(ultimaDetecao);
      display.setCursor(0, 54);
#if USAR_BLYNK
      display.print(Blynk.connected() ? "Blynk: Ligado" : "Blynk: Desligado");
#else
      display.print("Modo: Demo offline");
#endif
    }
  }
  else if (paginaAtual == 1) {
    display.setTextSize(1);
    display.setCursor(0, 14);
    display.print("G:");
    display.print(gForceAtual, 2);
    display.print(" Gy:");
    display.print(gyroDpsAtual, 0);
    display.print("d/s");
    display.setCursor(0, 24);
    display.print("Bat:");
    display.print(voltagemBateria, 2);
    display.print("V ADC:");
    display.print(rawADC);
    display.setCursor(0, 34);
#if SAFE_FALL_TEM_EDGE_IMPULSE
    display.print("Socorro IA:");
    display.print((int)(probabilidadeSocorro * 100));
    display.print("%");
#else
    display.print("IA voz: ausente");
#endif
    display.setCursor(0, 44);
    display.print("Impacto lim:");
    display.print(limiarImpactoG, 2);
    display.print("G");
    display.setCursor(0, 54);
    if (estado == ESTADO_VALIDAR_QUEDA) display.print("FSM: validar queda");
    else display.print("FSM: monitorizar");
  }
  display.display();
}

// BATERIA — ADC
float lerTensaoBateria() {
  const int LEITURAS = 40;
  long soma = 0;

  for (int i = 0; i < LEITURAS; i++) {
    soma += analogRead(BATT_ADC_PIN);
    delay(2);
  }

  float adcMedio = soma / (float)LEITURAS;
  rawADC = (int)adcMedio;

  float tensaoNoPino = (adcMedio / 4095.0f) * BATT_VREF;
  float tensaoBateria = tensaoNoPino * BATT_FATOR_DIVISOR * BATT_CALIBRACAO;

  return tensaoBateria;
}

int calcularPercentagemBateria(float vbat) {
  if (vbat >= 4.20f) return 100;
  if (vbat <= 3.30f) return 0;

  return (int)((vbat - 3.30f) * 100.0f / (4.20f - 3.30f));
}

void atualizarBateria() {
  static unsigned long ultimaLeitura = 0;
  static float percFiltrada = -1.0f;

  if (millis() - ultimaLeitura < 1000) return;

  ultimaLeitura = millis();

  voltagemBateria = lerTensaoBateria();
  int p = calcularPercentagemBateria(voltagemBateria);

  if (percFiltrada < 0) percFiltrada = p;

  percFiltrada = percFiltrada * 0.90f + p * 0.10f;
  percentagemBateria = constrain((int)roundf(percFiltrada), 0, 100);
}

// MPU6050
void calcularOrientacao(float ax_, float ay_, float az_, float &roll, float &pitch) {
  roll = atan2f(ay_, az_) * 180.0f / PI;
  pitch = atan2f(-ax_, sqrtf(ay_ * ay_ + az_ * az_)) * 180.0f / PI;
}

float diferencaAngular(float a, float b) {
  float d = fmodf(a - b + 540.0f, 360.0f) - 180.0f;
  return fabsf(d);
}

void iniciarMPU() {
  if (mpu.begin(0x68, &Wire)) {
    mpuOK = true;
  } else if (mpu.begin(0x69, &Wire)) {
    mpuOK = true;
  } else {
    mpuOK = false;
    Serial.println("ERRO: MPU6050 nao encontrado");
    return;
  }

  mpu.setAccelerometerRange(MPU6050_RANGE_8_G);
  mpu.setGyroRange(MPU6050_RANGE_500_DEG);
  mpu.setFilterBandwidth(MPU6050_BAND_44_HZ);

  Serial.println("MPU6050 OK");
}

void calibrarIMU() {
  if (!mpuOK) return;

  if (oledOK) {
    display.clearDisplay();
    display.setTextColor(SSD1306_WHITE);
    display.setTextSize(1);
    display.setCursor(12, 15);
    display.print("CALIBRACAO IMU");
    display.setCursor(5, 35);
    display.print("Manter imovel 3 s");
    display.display();
  }

  const int N = 150;

  double somaG = 0.0;
  double somaG2 = 0.0;
  double somaGyro = 0.0;
  double somaGyro2 = 0.0;
  double somaAx = 0.0;
  double somaAy = 0.0;
  double somaAz = 0.0;

  for (int i = 0; i < N; i++) {
    sensors_event_t a, g, temp;
    mpu.getEvent(&a, &g, &temp);

    float moduloG = sqrtf(
      a.acceleration.x * a.acceleration.x +
      a.acceleration.y * a.acceleration.y +
      a.acceleration.z * a.acceleration.z
    ) / 9.80665f;

    float moduloGyro = sqrtf(
      g.gyro.x * g.gyro.x +
      g.gyro.y * g.gyro.y +
      g.gyro.z * g.gyro.z
    ) * 180.0f / PI;

    somaG += moduloG;
    somaG2 += moduloG * moduloG;

    somaGyro += moduloGyro;
    somaGyro2 += moduloGyro * moduloGyro;

    somaAx += a.acceleration.x;
    somaAy += a.acceleration.y;
    somaAz += a.acceleration.z;

    delay(20);
  }

  gRepouso = somaG / N;

  float varianciaG = fmaxf(0.0f, somaG2 / N - gRepouso * gRepouso);
  ruidoG = sqrtf(varianciaG);

  float gyroMedio = somaGyro / N;
  float varianciaGyro = fmaxf(0.0f, somaGyro2 / N - gyroMedio * gyroMedio);
  float ruidoGyro = sqrtf(varianciaGyro);

  limiarImpactoG = constrain(gRepouso + fmaxf(1.25f, ruidoG * 8.0f), 2.25f, 3.10f);
  limiarQuedaLivreG = constrain(gRepouso - 0.48f, 0.42f, 0.58f);
  toleranciaImobilidadeG = constrain(fmaxf(0.10f, ruidoG * 5.0f), 0.10f, 0.20f);
  limiarImobilidadeGyroDps = constrain(fmaxf(18.0f, gyroMedio + ruidoGyro * 5.0f), 18.0f, 38.0f);

  calcularOrientacao(somaAx / N, somaAy / N, somaAz / N, rollReferencia, pitchReferencia);

  Serial.println("=== CALIBRACAO IMU CONSERVADORA ===");
  Serial.print("gRepouso: ");
  Serial.println(gRepouso, 3);

  Serial.print("limiarImpactoG: ");
  Serial.println(limiarImpactoG, 2);

  Serial.print("limiarQuedaLivreG: ");
  Serial.println(limiarQuedaLivreG, 2);

  Serial.print("toleranciaImobilidadeG: ");
  Serial.println(toleranciaImobilidadeG, 2);

  Serial.print("limiarImobilidadeGyroDps: ");
  Serial.println(limiarImobilidadeGyroDps, 1);

  if (oledOK) {
    display.clearDisplay();
    display.setTextColor(SSD1306_WHITE);
    display.setTextSize(1);
    display.setCursor(0, 18);
    display.print("IMU calibrado");
    display.setCursor(0, 36);
    display.display();
    delay(700);
  }
}

void amostrarIMU() {
  if (!mpuOK) return;
  if (millis() - ultimaAmostraIMUMs < PERIODO_IMU_MS) return;

  ultimaAmostraIMUMs = millis();

  sensors_event_t a, g, temp;
  mpu.getEvent(&a, &g, &temp);

  ax = a.acceleration.x;
  ay = a.acceleration.y;
  az = a.acceleration.z;

  gx = g.gyro.x;
  gy = g.gyro.y;
  gz = g.gyro.z;

  gForceAtual = sqrtf(ax * ax + ay * ay + az * az) / 9.80665f;
  gyroDpsAtual = sqrtf(gx * gx + gy * gy + gz * gz) * 180.0f / PI;
  temperaturaAtual = temp.temperature;

  calcularOrientacao(ax, ay, az, rollAtual, pitchAtual);
}

// MICROFONE
void iniciarMicrofone() {
  i2s_chan_config_t chan_cfg = I2S_CHANNEL_DEFAULT_CONFIG(I2S_PORT, I2S_ROLE_MASTER);

  esp_err_t err = i2s_new_channel(&chan_cfg, NULL, &i2sMicHandle);

  if (err != ESP_OK) {
    Serial.print("Erro i2s_new_channel: ");
    Serial.println(err);
    micOK = false;
    return;
  }

  i2s_std_config_t std_cfg = {
    .clk_cfg = I2S_STD_CLK_DEFAULT_CONFIG(16000),
    .slot_cfg = I2S_STD_PHILIPS_SLOT_DEFAULT_CONFIG(I2S_DATA_BIT_WIDTH_32BIT, I2S_SLOT_MODE_MONO),
    .gpio_cfg = {
      .mclk = I2S_GPIO_UNUSED,
      .bclk = (gpio_num_t)I2S_SCK,
      .ws = (gpio_num_t)I2S_WS,
      .dout = I2S_GPIO_UNUSED,
      .din = (gpio_num_t)I2S_SD,
      .invert_flags = {
        .mclk_inv = false,
        .bclk_inv = false,
        .ws_inv = false,
      },
    },
  };

  // Garante leitura do canal esquerdo (equivalente ao antigo ONLY_LEFT)
  std_cfg.slot_cfg.slot_mask = I2S_STD_SLOT_LEFT;

  err = i2s_channel_init_std_mode(i2sMicHandle, &std_cfg);

  if (err != ESP_OK) {
    Serial.print("Erro i2s_channel_init_std_mode: ");
    Serial.println(err);
    micOK = false;
    return;
  }

  err = i2s_channel_enable(i2sMicHandle);

  if (err != ESP_OK) {
    Serial.print("Erro i2s_channel_enable: ");
    Serial.println(err);
    micOK = false;
    return;
  }

  micOK = true;
  Serial.println("Microfone SPH0645 OK (driver I2S novo)");
}

void lerMicrofoneNivel() {
  if (!micOK) return;

  const int samples = 256;
  int32_t buffer[samples];
  size_t bytes_read = 0;

  esp_err_t err = i2s_channel_read(
    i2sMicHandle,
    buffer,
    sizeof(buffer),
    &bytes_read,
    50 / portTICK_PERIOD_MS
  );

  if (err != ESP_OK || bytes_read == 0) {
    micNivelMedio = 0;
    micPico = 0;
    return;
  }

  int n = bytes_read / sizeof(int32_t);

  long long soma = 0;
  int32_t maxAbs = 0;

  for (int i = 0; i < n; i++) {
    int32_t sample = buffer[i] >> 14;
    int32_t absSample = sample < 0 ? -sample : sample;

    soma += absSample;

    if (absSample > maxAbs) {
      maxAbs = absSample;
    }
  }

  micNivelMedio = soma / n;
  micPico = maxAbs;
}

#if SAFE_FALL_TEM_EDGE_IMPULSE
int get_signal_data(size_t offset, size_t length, float* out_ptr) {
  size_t bytes_read = 0;
  static int32_t i2s_raw_buf[256];
  size_t samples_processed = 0;

  while (samples_processed < length) {
    size_t to_read = length - samples_processed;

    if (to_read > 256) {
      to_read = 256;
    }

    i2s_channel_read(
      i2sMicHandle,
      (char*)i2s_raw_buf,
      to_read * sizeof(int32_t),
      &bytes_read,
      100 / portTICK_PERIOD_MS
    );

    size_t samples_read = bytes_read / sizeof(int32_t);

    if (samples_read > 0) {
      for (size_t i = 0; i < samples_read; i++) {
        int16_t sample16 = (int16_t)(i2s_raw_buf[i] >> 14);
        out_ptr[samples_processed + i] = (float)sample16;
      }

      samples_processed += samples_read;
      amostrarIMU();
      yield();

    } else {
      for (size_t i = samples_processed; i < length; i++) {
        out_ptr[i] = 0.0f;
      }

      break;
    }
  }

  return 0;
}
#endif

// ALERTAS
void enviarAlertaRemoto(const String& mensagem) {
#if USAR_BLYNK
  if (Blynk.connected()) {
    Blynk.virtualWrite(V1, 1);
    Blynk.logEvent("queda_detetada", mensagem);   // <-- dispara o evento = envia email
    Serial.print("[BLYNK] Evento disparado: ");
    Serial.println(mensagem);
  } else {
    Serial.println("[BLYNK] Alerta NAO enviado - sem ligacao Blynk");
  }
#else
  (void)mensagem;
#endif
}

void limparAlertaRemoto() {
#if USAR_BLYNK
  if (Blynk.connected()) {
    Blynk.virtualWrite(V1, 0);
  }
#endif
}

void iniciarPreAlerta(const String& origem) {
  if (estado == ESTADO_PRE_ALERTA || estado == ESTADO_ALERTA) return;

  estado = ESTADO_PRE_ALERTA;
  origemAlerta = origem;
  inicioPreAlertaMs = millis();
  alertaEnviado = false;
  ultimaDetecao = origem;
  cancelamentoConfirmado = false;

  Serial.print("[PRE-ALERTA] ");
  Serial.println(origem);

  bipCurto();
}

void confirmarAlerta() {
  if (estado != ESTADO_PRE_ALERTA) return;

  estado = ESTADO_ALERTA;
  alertaEnviado = true;

  String msg = "SAFE-FALL: " + origemAlerta;
  enviarAlertaRemoto(msg);

  Serial.print("[ALERTA] Confirmado: ");
  Serial.println(origemAlerta);
}

void cancelarAlerta(const String& motivo) {
  estado = ESTADO_CANCELADO;
  alertaEnviado = false;
  origemAlerta = "";
  ultimaDetecao = motivo;
  bloquearVozAteMs = millis() + 4000;

  limparAlertaRemoto();

  buzzerOff();
  bipDuplo();

  Serial.print("[CANCELADO] ");
  Serial.println(motivo);
}

// BOTÃO
void atualizarBotao() {
  bool botaoAtual = digitalRead(BOTAO_PIN);

  // Botao acabou de ser pressionado
  if (botaoAtual == LOW && botaoAnterior == HIGH) {
    inicioPressaoBotaoMs = millis();
    cancelamentoConfirmado = false;
  }

  // Botao mantido pressionado
  if (botaoAtual == LOW) {
    unsigned long tempoPressionado = millis() - inicioPressaoBotaoMs;

    // Cancelar alerta com pressao longa
    if (tempoPressionado >= TEMPO_CANCELAR_BOTAO_MS && !cancelamentoConfirmado) {
      cancelamentoConfirmado = true;

      if (estado == ESTADO_PRE_ALERTA || estado == ESTADO_ALERTA || estado == ESTADO_VALIDAR_QUEDA) {
        cancelarAlerta("Botao premido");
      }
    }
  }

  // Botao foi solto
  if (botaoAtual == HIGH && botaoAnterior == LOW) {
    unsigned long duracao = millis() - inicioPressaoBotaoMs;

    Serial.print("[BOTAO] Duracao: ");
    Serial.print(duracao);
    Serial.println(" ms");

    if (estado == ESTADO_CANCELADO) {
      estado = ESTADO_NORMAL;
    }

    // Troca pagina com clique curto ou medio, desde que nao esteja em alerta
    else if (
      estado == ESTADO_NORMAL &&
      duracao >= 30 &&
      duracao < 1500
    ) {
      paginaAtual = (paginaAtual + 1) % TOTAL_PAGINAS;
      ultimoDisplayMs = 0;
      Serial.print("[OLED] Pagina alterada para: ");
      Serial.println(paginaAtual);
    }
  }

  botaoAnterior = botaoAtual;
}

// DETEÇÃO DE QUEDA
void iniciarValidacaoQueda() {
  estado = ESTADO_VALIDAR_QUEDA;
  inicioValidacaoQuedaMs = millis();
  imovelDesdeMs = 0;

  eventoHouveQuedaLivre = (millis() - ultimaQuedaLivreMs <= 1000);
  eventoHouveRotacao = (millis() - ultimaRotacaoForteMs <= 1000);
  eventoPicoImpactoG = gForceAtual;
  eventoPicoGyroDps = gyroDpsAtual;
  eventoMaiorMudancaOrientacao = 0.0f;
  eventoRollAntes = rollReferencia;
  eventoPitchAntes = pitchReferencia;

  Serial.println("[QUEDA?] Validacao conservadora iniciada");
  bipCurto();
}

void resetValidacaoQueda() {
  estado = ESTADO_NORMAL;
  inicioValidacaoQuedaMs = 0;
  imovelDesdeMs = 0;
  eventoHouveQuedaLivre = false;
  eventoHouveRotacao = false;
  eventoPicoImpactoG = 0.0f;
  eventoPicoGyroDps = 0.0f;
  eventoMaiorMudancaOrientacao = 0.0f;
}

void atualizarEventosContexto() {
  unsigned long agora = millis();

  if (gForceAtual < limiarQuedaLivreG) {
    if (inicioBaixaAceleracaoMs == 0) inicioBaixaAceleracaoMs = agora;

    if (agora - inicioBaixaAceleracaoMs >= TEMPO_QUEDA_LIVRE_MIN_MS) {
      ultimaQuedaLivreMs = agora;
    }
  } else {
    inicioBaixaAceleracaoMs = 0;
  }

  if (gyroDpsAtual >= ROTACAO_FORTE_DPS) {
    if (inicioRotacaoForteMs == 0) inicioRotacaoForteMs = agora;

    if (agora - inicioRotacaoForteMs >= TEMPO_ROTACAO_FORTE_MIN_MS) {
      ultimaRotacaoForteMs = agora;
    }
  } else {
    inicioRotacaoForteMs = 0;
  }
}

void atualizarDetecaoQueda() {
  if (!mpuOK) return;
  if (estado == ESTADO_PRE_ALERTA || estado == ESTADO_ALERTA || estado == ESTADO_CANCELADO) return;

  unsigned long agora = millis();

  atualizarEventosContexto();

  if (estado == ESTADO_NORMAL) {
    bool estavel = fabsf(gForceAtual - gRepouso) <= toleranciaImobilidadeG &&
                   gyroDpsAtual <= limiarImobilidadeGyroDps;

    if (!orientacaoRapidaInicializada) {
      rollAnteriorRapido = rollAtual;
      pitchAnteriorRapido = pitchAtual;
      orientacaoRapidaInicializada = true;
    }

    float mudancaRapidaRoll = diferencaAngular(rollAtual, rollAnteriorRapido);
    float mudancaRapidaPitch = diferencaAngular(pitchAtual, pitchAnteriorRapido);
    float mudancaRapidaOrientacao = fmaxf(mudancaRapidaRoll, mudancaRapidaPitch);

    rollAnteriorRapido = rollAtual;
    pitchAnteriorRapido = pitchAtual;

    if (estavel) {
      rollReferencia = 0.98f * rollReferencia + 0.02f * rollAtual;
      pitchReferencia = 0.98f * pitchReferencia + 0.02f * pitchAtual;
    }

    bool quedaLivreRecente = (agora - ultimaQuedaLivreMs) <= 1200;
    bool rotacaoRecente = (agora - ultimaRotacaoForteMs) <= 1200;

    bool impactoForte = gForceAtual >= limiarImpactoG;
    bool impactoDireto = gForceAtual >= IMPACTO_DIRETO_MIN_G;

    bool movimentoCompativelComQueda =
      quedaLivreRecente ||
      rotacaoRecente ||
      gyroDpsAtual >= GIRO_DIRETO_MIN_DPS ||
      mudancaRapidaOrientacao >= MUDANCA_RAPIDA_MIN_GRAUS;

    // Caso 1: queda clássica — impacto forte com queda livre ou rotação recente
    if (impactoForte && (quedaLivreRecente || rotacaoRecente)) {
      janelaQuedaDiretaAtiva = false;
      inicioParadoAposImpactoMs = 0;
      iniciarPreAlerta("QUEDA DETETADA");
      return;
    }

    // Caso 1b: impacto forte sem contexto prévio (ex: queda de costas sem rotação)
    if (impactoForte && !janelaQuedaDiretaAtiva) {
      janelaQuedaDiretaAtiva = true;
      inicioJanelaQuedaDiretaMs = agora;
      inicioParadoAposImpactoMs = 0;
      picoQuedaDiretaG = gForceAtual;

      Serial.print("[IMPACTO-FORTE] Sem contexto previo, a validar paragem. G=");
      Serial.println(gForceAtual, 2);
    }

    // Caso 2: impacto direto com movimento compatível com queda
    if (!janelaQuedaDiretaAtiva && (impactoDireto && movimentoCompativelComQueda)) {
      janelaQuedaDiretaAtiva = true;
      inicioJanelaQuedaDiretaMs = agora;
      inicioParadoAposImpactoMs = 0;
      picoQuedaDiretaG = gForceAtual;

      Serial.print("[IMPACTO] Janela queda direta. G=");
      Serial.print(gForceAtual, 2);
      Serial.print(" Gyro=");
      Serial.print(gyroDpsAtual, 1);
      Serial.print(" MudOri=");
      Serial.println(mudancaRapidaOrientacao, 1);
    }

    // Confirma queda se depois do impacto a pulseira ficar parada
    if (janelaQuedaDiretaAtiva) {
      bool dentroDaJanela = agora - inicioJanelaQuedaDiretaMs <= JANELA_QUEDA_DIRETA_MS;

      // ALTERAÇÃO: fator de tolerância 2.2 → 3.0 e gyro 1.8 → 2.2
      bool paradoDepoisImpacto =
        fabsf(gForceAtual - gRepouso) <= toleranciaImobilidadeG * 3.0f &&
        gyroDpsAtual <= limiarImobilidadeGyroDps * 2.2f;

      if (gForceAtual > picoQuedaDiretaG) {
        picoQuedaDiretaG = gForceAtual;
      }

      if (dentroDaJanela && paradoDepoisImpacto) {
        if (inicioParadoAposImpactoMs == 0) {
          inicioParadoAposImpactoMs = agora;
        }

        // ALTERAÇÃO: TEMPO_PARADO_POS_IMPACTO_MS agora é 700ms (era 500ms)
        if (agora - inicioParadoAposImpactoMs >= TEMPO_PARADO_POS_IMPACTO_MS) {
          janelaQuedaDiretaAtiva = false;
          inicioParadoAposImpactoMs = 0;

          Serial.print("[QUEDA] Impacto + paragem confirmada. Pico=");
          Serial.println(picoQuedaDiretaG, 2);

          iniciarPreAlerta("QUEDA DETETADA");
          return;
        }
      }

      else {
        inicioParadoAposImpactoMs = 0;
      }

      if (!dentroDaJanela) {
        janelaQuedaDiretaAtiva = false;
        inicioParadoAposImpactoMs = 0;
        Serial.println("[IGNORADO] Impacto sem paragem dentro da janela");
      }
    }

    return;
  }

  if (estado == ESTADO_VALIDAR_QUEDA) {
    eventoPicoImpactoG = fmaxf(eventoPicoImpactoG, gForceAtual);
    eventoPicoGyroDps = fmaxf(eventoPicoGyroDps, gyroDpsAtual);
    eventoHouveRotacao = eventoHouveRotacao || ((agora - ultimaRotacaoForteMs) <= 1000);
    eventoHouveQuedaLivre = eventoHouveQuedaLivre || ((agora - ultimaQuedaLivreMs) <= 1000);

    float mudancaRoll = diferencaAngular(rollAtual, eventoRollAntes);
    float mudancaPitch = diferencaAngular(pitchAtual, eventoPitchAntes);

    eventoMaiorMudancaOrientacao = fmaxf(
      eventoMaiorMudancaOrientacao,
      fmaxf(mudancaRoll, mudancaPitch)
    );

    bool imovel = fabsf(gForceAtual - gRepouso) <= toleranciaImobilidadeG &&
                  gyroDpsAtual <= limiarImobilidadeGyroDps;

    if (imovel) {
      if (imovelDesdeMs == 0) imovelDesdeMs = agora;
    } else if (gyroDpsAtual > limiarImobilidadeGyroDps * 1.8f ||
               fabsf(gForceAtual - gRepouso) > toleranciaImobilidadeG * 2.0f) {
      imovelDesdeMs = 0;
    }

    bool imobilidadeConfirmada = imovelDesdeMs > 0 &&
                                 agora - imovelDesdeMs >= TEMPO_IMOBILIDADE_MS;

    bool orientacaoMudou = eventoMaiorMudancaOrientacao >= MUDANCA_ORIENTACAO_MIN_GRAUS;
    bool impactoSevero = eventoPicoImpactoG >= limiarImpactoG + 0.45f;

    int evidencia = 0;

    if (eventoHouveQuedaLivre) evidencia += 2;
    if (eventoHouveRotacao) evidencia += 2;
    if (orientacaoMudou) evidencia += 2;
    if (impactoSevero) evidencia += 2;
    if (imobilidadeConfirmada) evidencia += 2;

    bool quedaConfirmada =
      imobilidadeConfirmada &&
      orientacaoMudou &&
      impactoSevero &&
      (eventoHouveQuedaLivre || eventoHouveRotacao) &&
      evidencia >= 8;

    if (quedaConfirmada) {
      Serial.print("[QUEDA] Confirmada. Evidencia=");
      Serial.println(evidencia);
      iniciarPreAlerta("QUEDA CONFIRMADA");
      return;
    }

    if (agora - inicioValidacaoQuedaMs >= JANELA_VALIDACAO_QUEDA_MS) {
      Serial.print("[IGNORADO] Movimento brusco. Evidencia=");
      Serial.println(evidencia);
      ultimaDetecao = "Mov. brusco ignorado";
      resetValidacaoQueda();
    }
  }
}

// VOZ — "SOCORRO"
#if SAFE_FALL_TEM_EDGE_IMPULSE
float obterProbabilidadeSocorro(const ei_impulse_result_t& result, float& maiorOutra) {
  float prob = 0.0f;
  maiorOutra = 0.0f;

  for (size_t i = 0; i < EI_CLASSIFIER_LABEL_COUNT; i++) {
    String label = result.classification[i].label;
    float p = result.classification[i].value;

    if (label.equalsIgnoreCase("socorro")) {
      prob = p;
    } else {
      maiorOutra = fmaxf(maiorOutra, p);
    }
  }

  return prob;
}

void processarComandoSocorroIA() {
  if (millis() < bloquearVozAteMs) return;
  if (estado != ESTADO_NORMAL) return;
  if (!micOK) return;

  static uint8_t votosSocorro = 0;
  static unsigned long primeiroVotoMs = 0;

  if (micPico < MIC_PICO_MIN_SOCORRO) {
    if (millis() - primeiroVotoMs > JANELA_VOTOS_SOCORRO_MS) {
      votosSocorro = 0;
    }
    return;
  }

  signal_t signal;
  signal.total_length = EI_CLASSIFIER_RAW_SAMPLE_COUNT;
  signal.get_data = &get_signal_data;

  ei_impulse_result_t result = { 0 };
  EI_IMPULSE_ERROR erro = run_classifier(&signal, &result, false);

  if (erro != EI_IMPULSE_OK) {
    Serial.print("[IA] Erro classificador: ");
    Serial.println(erro);
    return;
  }

  float maiorOutra = 0.0f;
  float prob = obterProbabilidadeSocorro(result, maiorOutra);
  probabilidadeSocorro = prob;

  bool margemValida = prob >= maiorOutra + MARGEM_VOZ;

  Serial.print("[IA] socorro=");
  Serial.print(prob, 2);
  Serial.print(" outra=");
  Serial.print(maiorOutra, 2);
  Serial.print(" mic=");
  Serial.print(micPico);
  Serial.print(" margem=");
  Serial.print(margemValida ? "OK" : "NAO");
  Serial.print(" votos=");
  Serial.println(votosSocorro);

  if (prob >= PROB_SOCORRO_FORTE && margemValida) {
    votosSocorro = 0;
    iniciarPreAlerta("VOZ: SOCORRO");
    return;
  }

  if (prob >= PROB_SOCORRO_VOTO && margemValida) {
    if (votosSocorro == 0 || millis() - primeiroVotoMs > JANELA_VOTOS_SOCORRO_MS) {
      votosSocorro = 0;
      primeiroVotoMs = millis();
    }

    votosSocorro++;

    if (votosSocorro >= VOTOS_SOCORRO_NECESSARIOS) {
      votosSocorro = 0;
      iniciarPreAlerta("VOZ: SOCORRO");
      return;
    }
  }

  else {
    if (millis() - primeiroVotoMs > JANELA_VOTOS_SOCORRO_MS) {
      votosSocorro = 0;
    }
  }
}
#endif

void atualizarVozOuSom() {
#if SAFE_FALL_TEM_EDGE_IMPULSE
  static unsigned long ultimaIA = 0;

  if (estado != ESTADO_NORMAL) return;
  if (!micOK) return;

  if (millis() - ultimaIA >= 800) {
    ultimaIA = millis();

    Serial.println("[IA] A ouvir agora: diga SOCORRO");
    processarComandoSocorroIA();
  }
#else
  // Sem Edge Impulse não há deteção vocal
#endif
}

// ESTADO ALERTA
void atualizarEstadoSistema() {
  if (estado == ESTADO_PRE_ALERTA) {
    if (millis() - inicioPreAlertaMs >= TEMPO_PRE_ALERTA_MS) {
      confirmarAlerta();
    }
  }

  atualizarBuzzerAlerta();
}

// SETUP
void setup() {
  Serial.begin(115200);
  delay(1500);

  pinMode(BUZZER_PIN, OUTPUT);
  pinMode(BOTAO_PIN, INPUT_PULLUP);
  delay(100);
  botaoEstavel = digitalRead(BOTAO_PIN);
  ultimaLeituraBotao = botaoEstavel;
  botaoAnterior = botaoEstavel;
  delay(100);
  botaoAnterior = digitalRead(BOTAO_PIN);
  pinMode(BATT_ADC_PIN, INPUT);

  analogReadResolution(12);
  analogSetPinAttenuation(BATT_ADC_PIN, ADC_11db);

  buzzerOff();

  delay(1000);

  limparBarramentoI2C();

  Wire.begin(I2C_SDA, I2C_SCL);
  Wire.setClock(100000);
  Wire.setTimeOut(150);
  delay(300);

  iniciarOLED();

  iniciarMPU();
  iniciarMicrofone();
  atualizarBateria();

  if (mpuOK) {
    calibrarIMU();
  }

#if USAR_BLYNK
  iniciarBlynkOnline();
#endif

  Serial.println("=== SAFE-FALL FINAL V11 - VOZ E QUEDA MELHORADAS ===");

  Serial.print("OLED: ");
  Serial.println(oledOK ? "OK" : "ERRO");

  Serial.print("MPU: ");
  Serial.println(mpuOK ? "OK" : "ERRO");

  Serial.print("MIC: ");
  Serial.println(micOK ? "OK" : "ERRO");

  Serial.print("Bateria: ");
  Serial.print(voltagemBateria, 2);
  Serial.println(" V");

  Serial.println("Detecao queda: FSM multimodal + caso 1b (impacto forte sem contexto)");
  Serial.println("Voz: limiares reduzidos, 2 votos, janela 6s, mic 1200");

#if SAFE_FALL_TEM_EDGE_IMPULSE
  Serial.println("Voz: Edge Impulse ativo - apenas palavra SOCORRO");
#else
  Serial.println("Voz: Edge Impulse ausente - sem detecao vocal");
#endif

#if USAR_BLYNK
  Serial.println("Blynk: modo online com BlynkSimpleEsp32");
#else
  Serial.println("Blynk: desligado");
#endif

  bipCurto();
  delay(1000);
}

// LOOP
void loop() {
#if USAR_BLYNK
  gerirBlynkOnline();
#endif

  amostrarIMU();
  lerMicrofoneNivel();
  atualizarBateria();
  atualizarBotao();
  atualizarDetecaoQueda();
  atualizarVozOuSom();
  atualizarEstadoSistema();
  atualizarOLED();

#if USAR_BLYNK
  enviarDadosBlynk();
#endif

  static unsigned long ultimoSerial = 0;

  if (millis() - ultimoSerial >= 500) {
    ultimoSerial = millis();

    Serial.print("Estado: ");
    Serial.print(nomeEstado());

    Serial.print(" | Bat: ");
    Serial.print(voltagemBateria, 2);
    Serial.print("V ");
    Serial.print(percentagemBateria);
    Serial.print("%");

    Serial.print(" | ADC: ");
    Serial.print(rawADC);

    Serial.print(" | G: ");
    Serial.print(gForceAtual, 2);

    Serial.print(" | Gyro: ");
    Serial.print(gyroDpsAtual, 1);

    Serial.print(" | Mic: ");
    Serial.print(micPico);

    Serial.print(" | Socorro: ");
    Serial.print((int)(probabilidadeSocorro * 100));
    Serial.print("%");

#if USAR_BLYNK
    Serial.print(" | WiFi: ");
    Serial.print(WiFi.status() == WL_CONNECTED ? "OK" : "NAO");

    Serial.print(" | Blynk: ");
    Serial.print(Blynk.connected() ? "OK" : "NAO");
#endif

    Serial.print(" | Pag: ");
    Serial.print(paginaAtual);
    Serial.print(" | Botao: ");
    Serial.println(botaoEstavel == LOW ? "PREMIDO" : "SOLTO");
  }

  delay(10);
}