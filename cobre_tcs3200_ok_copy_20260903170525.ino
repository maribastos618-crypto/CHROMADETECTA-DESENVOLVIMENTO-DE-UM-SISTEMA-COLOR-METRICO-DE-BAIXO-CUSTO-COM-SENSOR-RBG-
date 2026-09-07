
/*
   CHROMADETECTA - TCS3200/GY-31
   MODO DE NOVA CALIBRACAO E ANALISE DE SENSIBILIDADE

   OBJETIVO:
   - Medir R, G e B
   - Calcular:
       1) R/(R+G+B)  -> METRICA PRINCIPAL
       2) B/(R+G+B)  -> METRICA AUXILIAR
       3) B/R        -> METRICA AUXILIAR
   - Nao usar a calibracao antiga para estimar concentracao.
   - Observar a tendencia dos sinais nas novas concentracoes.
   - Mostrar media, desvio-padrao e faixa das leituras.
   - Medir branco da sessao para acompanhar variacoes do sistema.

   PROTOCOLO:
   1. Coloque branco (agua + glutamato).
   2. Digite BRANCO.
   3. Prepare uma concentracao.
   4. Adicione glutamato e misture sempre da mesma forma.
   5. Digite OK.
   6. Aguarde 10 s.
   7. O Arduino mede durante 10 s.
   8. O resultado mostra as metricas.

   IMPORTANTE:
   A concentracao NAO e calculada neste momento.
   Os resultados servem para construir uma NOVA calibracao.
*/

// ============================================================
// PINOS
// ============================================================

const byte PIN_S0  = 4;
const byte PIN_S1  = 5;
const byte PIN_S2  = 6;
const byte PIN_S3  = 7;
const byte PIN_OUT = 8;


// ============================================================
// TEMPOS
// ============================================================

const unsigned long REACTION_WAIT_MS = 10000UL;
const unsigned long MEASUREMENT_WINDOW_MS = 10000UL;


// ============================================================
// SENSOR
// ============================================================

// 20% de frequencia do TCS3200
const byte PULSE_SCALING_S0 = HIGH;
const byte PULSE_SCALING_S1 = LOW;

// Quantidade de leituras internas por bloco
const byte SAMPLES_PER_READING = 25;

// Tempo maximo esperando o pulso
const unsigned long PULSE_TIMEOUT_US = 100000UL;

// Quantidade maxima de blocos durante a janela
const byte MAX_WINDOW_READINGS = 15;


// ============================================================
// FILTRAGEM
// ============================================================

// Em vez de remover agressivamente valores,
// usamos um limite bem amplo.
// O objetivo agora e OBSERVAR a variacao real.
const float OUTLIER_SIGMA_LIMIT = 3.0;


// ============================================================
// ESTADOS DO SISTEMA
// ============================================================

enum SystemState {
  IDLE,
  WAITING_FOR_REACTION,
  READING_WINDOW
};

enum MeasurementType {
  SAMPLE_MEASUREMENT,
  BLANK_MEASUREMENT
};

SystemState systemState = IDLE;
MeasurementType currentMeasurementType = SAMPLE_MEASUREMENT;

unsigned long stateStartedAt = 0;


// ============================================================
// BRANCO DA SESSAO
// ============================================================

float blankR = 0.0;
float blankG = 0.0;
float blankB = 0.0;

float blankRFraction = 0.0;
float blankBFraction = 0.0;
float blankBOverR = 0.0;

bool hasSessionBlank = false;


// ============================================================
// ESTRUTURA DE LEITURA
// ============================================================

struct ColorReading {

  float r;
  float g;
  float b;

  float rFraction;
  float bFraction;
  float bOverR;

  float rFractionSD;
  float bFractionSD;
  float bOverRSD;

  byte acceptedSamples;
};


// ============================================================
// JANELA DE MEDICAO
// ============================================================

float windowR[MAX_WINDOW_READINGS];
float windowG[MAX_WINDOW_READINGS];
float windowB[MAX_WINDOW_READINGS];

float windowRFraction[MAX_WINDOW_READINGS];
float windowBFraction[MAX_WINDOW_READINGS];
float windowBOverR[MAX_WINDOW_READINGS];

byte windowCount = 0;


// ============================================================
// SERIAL
// ============================================================

const byte COMMAND_BUFFER_SIZE = 20;

char commandBuffer[COMMAND_BUFFER_SIZE];
byte commandLength = 0;


// ============================================================
// SETUP
// ============================================================

void setup() {

  Serial.begin(115200);

  pinMode(PIN_S0, OUTPUT);
  pinMode(PIN_S1, OUTPUT);
  pinMode(PIN_S2, OUTPUT);
  pinMode(PIN_S3, OUTPUT);
  pinMode(PIN_OUT, INPUT);

  digitalWrite(PIN_S0, PULSE_SCALING_S0);
  digitalWrite(PIN_S1, PULSE_SCALING_S1);

  Serial.println();
  Serial.println(F("=========================================="));
  Serial.println(F("       CHROMADETECTA - NOVA CALIBRACAO"));
  Serial.println(F("       TCS3200 / GY-31"));
  Serial.println(F("=========================================="));
  Serial.println();

  Serial.println(F("METRICA PRINCIPAL: R/(R+G+B)"));
  Serial.println(F("METRICAS AUXILIARES: B/(R+G+B) e B/R"));
  Serial.println();

  Serial.println(F("Comandos:"));
  Serial.println(F("BRANCO    -> mede o branco da sessao"));
  Serial.println(F("OK        -> inicia medicao"));
  Serial.println(F("CANCELAR  -> cancela medicao"));
  Serial.println(F("AJUDA     -> mostra ajuda"));
  Serial.println();

  Serial.println(F("IMPORTANTE: nenhuma calibracao antiga sera usada."));
  Serial.println(F("O objetivo agora e analisar a tendencia dos sinais."));
  Serial.println();
}


// ============================================================
// LOOP
// ============================================================

void loop() {

  readSerialCommands();


  // ----------------------------------------------------------
  // ESPERA DA REACAO
  // ----------------------------------------------------------

  if (systemState == WAITING_FOR_REACTION) {

    if (millis() - stateStartedAt >= REACTION_WAIT_MS) {

      beginReadingWindow();
    }

    return;
  }


  // ----------------------------------------------------------
  // JANELA DE LEITURA
  // ----------------------------------------------------------

  if (systemState == READING_WINDOW) {

    bool timeFinished =
      millis() - stateStartedAt >= MEASUREMENT_WINDOW_MS;

    bool countFinished =
      windowCount >= MAX_WINDOW_READINGS;


    if (timeFinished || countFinished) {

      finishMeasurement();

      return;
    }


    ColorReading reading = readFilteredColor();


    windowR[windowCount] = reading.r;
    windowG[windowCount] = reading.g;
    windowB[windowCount] = reading.b;

    windowRFraction[windowCount] = reading.rFraction;
    windowBFraction[windowCount] = reading.bFraction;
    windowBOverR[windowCount] = reading.bOverR;

    windowCount++;


    printLiveReading(reading, windowCount);
  }
}


// ============================================================
// LEITURA DOS COMANDOS
// ============================================================

void readSerialCommands() {

  while (Serial.available() > 0) {

    char incoming = Serial.read();


    if (incoming == '\r' || incoming == '\n') {

      if (commandLength > 0) {

        commandBuffer[commandLength] = '\0';

        executeCommand();

        commandLength = 0;
      }

      continue;
    }


    if (commandLength < COMMAND_BUFFER_SIZE - 1) {

      commandBuffer[commandLength] = incoming;

      commandLength++;
    }
  }
}


// ============================================================
// EXECUTA COMANDO
// ============================================================

void executeCommand() {

  if (commandIs("OK")) {

    startProtocol(SAMPLE_MEASUREMENT);

    return;
  }


  if (commandIs("BRANCO")) {

    startProtocol(BLANK_MEASUREMENT);

    return;
  }


  if (commandIs("CANCELAR")) {

    cancelProtocol();

    return;
  }


  if (commandIs("AJUDA")) {

    printHelp();

    return;
  }


  Serial.println(F("Comando nao reconhecido."));
  Serial.println(F("Use BRANCO, OK, CANCELAR ou AJUDA."));
}


// ============================================================
// COMPARACAO DE COMANDO
// ============================================================

bool commandIs(const char target[]) {

  byte i = 0;


  while (target[i] != '\0') {

    char typed = commandBuffer[i];


    if (typed == '\0') {

      return false;
    }


    if (typed >= 'a' && typed <= 'z') {

      typed = typed - ('a' - 'A');
    }


    if (typed != target[i]) {

      return false;
    }


    i++;
  }


  return commandBuffer[i] == '\0';
}


// ============================================================
// INICIA PROTOCOLO
// ============================================================

void startProtocol(MeasurementType measurementType) {

  if (systemState != IDLE) {

    Serial.println(
      F("Ja existe uma medicao em andamento. Use CANCELAR.")
    );

    return;
  }


  currentMeasurementType = measurementType;

  systemState = WAITING_FOR_REACTION;

  stateStartedAt = millis();


  if (measurementType == BLANK_MEASUREMENT) {

    Serial.println();
    Serial.println(F("=========================================="));
    Serial.println(F("BRANCO DA SESSAO"));
    Serial.println(F("Aguardando 10 segundos..."));
    Serial.println(F("=========================================="));
  }

  else {

    Serial.println();
    Serial.println(F("=========================================="));
    Serial.println(F("NOVA AMOSTRA"));
    Serial.println(F("Aguardando 10 segundos para reacao..."));
    Serial.println(F("=========================================="));
  }
}


// ============================================================
// CANCELAR
// ============================================================

void cancelProtocol() {

  if (systemState == IDLE) {

    Serial.println(F("Nenhuma medicao em andamento."));

    return;
  }


  systemState = IDLE;

  windowCount = 0;

  Serial.println(F("Medicao cancelada."));
}


// ============================================================
// AJUDA
// ============================================================

void printHelp() {

  Serial.println();
  Serial.println(F("BRANCO: mede o branco da sessao."));
  Serial.println(F("OK: inicia uma nova amostra."));
  Serial.println(F("CANCELAR: cancela a medicao."));
  Serial.println(F("AJUDA: mostra esta mensagem."));
  Serial.println();

  Serial.println(F("METRICAS:"));
  Serial.println(F("Rfrac = R/(R+G+B)"));
  Serial.println(F("Bfrac = B/(R+G+B)"));
  Serial.println(F("B/R    = B dividido por R"));
  Serial.println();
}


// ============================================================
// INICIA JANELA DE LEITURA
// ============================================================

void beginReadingWindow() {

  systemState = READING_WINDOW;

  stateStartedAt = millis();

  windowCount = 0;

  Serial.println();
  Serial.println(F("LENDO DURANTE 10 SEGUNDOS..."));
  Serial.println();
}


// ============================================================
// FINALIZA MEDICAO
// ============================================================

void finishMeasurement() {

  systemState = IDLE;


  if (windowCount == 0) {

    Serial.println(F("Falha: nenhuma leitura coletada."));

    return;
  }


  ColorReading finalReading =
    aggregateWindowReadings();


  // ----------------------------------------------------------
  // BRANCO
  // ----------------------------------------------------------

  if (currentMeasurementType == BLANK_MEASUREMENT) {

    blankR = finalReading.r;
    blankG = finalReading.g;
    blankB = finalReading.b;

    blankRFraction = finalReading.rFraction;
    blankBFraction = finalReading.bFraction;
    blankBOverR = finalReading.bOverR;

    hasSessionBlank = true;

    printBlankResult(finalReading);

    return;
  }


  // ----------------------------------------------------------
  // AMOSTRA
  // ----------------------------------------------------------

  printFinalSampleResult(finalReading);
}


// ============================================================
// LEITURA AO VIVO
// ============================================================

void printLiveReading(ColorReading reading, byte number) {

  Serial.print(F("LEITURA "));
  Serial.print(number);

  Serial.print(F(" | R="));
  Serial.print(reading.r, 7);

  Serial.print(F(" G="));
  Serial.print(reading.g, 7);

  Serial.print(F(" B="));
  Serial.print(reading.b, 7);

  Serial.print(F(" | Rfrac="));
  Serial.print(reading.rFraction, 7);

  Serial.print(F(" | Bfrac="));
  Serial.print(reading.bFraction, 7);

  Serial.print(F(" | B/R="));
  Serial.print(reading.bOverR, 7);

  Serial.print(F(" | n="));
  Serial.println(reading.acceptedSamples);
}


// ============================================================
// LEITURA FILTRADA
// ============================================================

ColorReading readFilteredColor() {

  float rValues[SAMPLES_PER_READING];
  float gValues[SAMPLES_PER_READING];
  float bValues[SAMPLES_PER_READING];

  float rFracValues[SAMPLES_PER_READING];
  float bFracValues[SAMPLES_PER_READING];
  float bOverRValues[SAMPLES_PER_READING];


  // ----------------------------------------------------------
  // COLETA 25 AMOSTRAS
  // ----------------------------------------------------------

  for (byte i = 0; i < SAMPLES_PER_READING; i++) {

    rValues[i] = readChannelIntensity('R');

    delay(8);

    gValues[i] = readChannelIntensity('G');

    delay(8);

    bValues[i] = readChannelIntensity('B');

    delay(8);


    rFracValues[i] =
      calculateRFraction(
        rValues[i],
        gValues[i],
        bValues[i]
      );


    bFracValues[i] =
      calculateBFraction(
        rValues[i],
        gValues[i],
        bValues[i]
      );


    bOverRValues[i] =
      calculateBOverR(
        rValues[i],
        bValues[i]
      );
  }


  // ----------------------------------------------------------
  // MEDIAS INICIAIS
  // ----------------------------------------------------------

  float meanRFrac =
    mean(rFracValues, SAMPLES_PER_READING);

  float meanBFrac =
    mean(bFracValues, SAMPLES_PER_READING);

  float meanBOverR =
    mean(bOverRValues, SAMPLES_PER_READING);


  float sdRFrac =
    standardDeviation(
      rFracValues,
      SAMPLES_PER_READING,
      meanRFrac
    );


  float sdBFrac =
    standardDeviation(
      bFracValues,
      SAMPLES_PER_READING,
      meanBFrac
    );


  float sdBOverR =
    standardDeviation(
      bOverRValues,
      SAMPLES_PER_READING,
      meanBOverR
    );


  // ----------------------------------------------------------
  // MEDIA DOS CANAIS
  // ----------------------------------------------------------

  float rSum = 0.0;
  float gSum = 0.0;
  float bSum = 0.0;

  float rFracSum = 0.0;
  float bFracSum = 0.0;
  float bOverRSum = 0.0;

  byte accepted = 0;


  // ----------------------------------------------------------
  // FILTRO
  // ----------------------------------------------------------

  for (byte i = 0; i < SAMPLES_PER_READING; i++) {

    bool keep = true;


    // Usa Rfrac como referencia principal.
    if (sdRFrac > 0.0) {

      keep =
        absFloat(rFracValues[i] - meanRFrac)
        <= OUTLIER_SIGMA_LIMIT * sdRFrac;
    }


    if (keep) {

      rSum += rValues[i];
      gSum += gValues[i];
      bSum += bValues[i];

      rFracSum += rFracValues[i];
      bFracSum += bFracValues[i];
      bOverRSum += bOverRValues[i];

      accepted++;
    }
  }


  // ----------------------------------------------------------
  // CASO EXTREMO
  // ----------------------------------------------------------

  if (accepted == 0) {

    accepted = SAMPLES_PER_READING;

    for (byte i = 0; i < SAMPLES_PER_READING; i++) {

      rSum += rValues[i];
      gSum += gValues[i];
      bSum += bValues[i];

      rFracSum += rFracValues[i];
      bFracSum += bFracValues[i];
      bOverRSum += bOverRValues[i];
    }
  }


  // ----------------------------------------------------------
  // RESULTADO
  // ----------------------------------------------------------

  ColorReading result;


  result.r = rSum / accepted;
  result.g = gSum / accepted;
  result.b = bSum / accepted;

  result.rFraction = rFracSum / accepted;
  result.bFraction = bFracSum / accepted;
  result.bOverR = bOverRSum / accepted;

  result.rFractionSD =
    standardDeviation(
      rFracValues,
      SAMPLES_PER_READING,
      result.rFraction
    );

  result.bFractionSD =
    standardDeviation(
      bFracValues,
      SAMPLES_PER_READING,
      result.bFraction
    );

  result.bOverRSD =
    standardDeviation(
      bOverRValues,
      SAMPLES_PER_READING,
      result.bOverR
    );

  result.acceptedSamples = accepted;


  return result;
}


// ============================================================
// AGREGA AS LEITURAS DA JANELA DE 10 SEGUNDOS
// ============================================================

ColorReading aggregateWindowReadings() {

  float meanRFrac =
    mean(windowRFraction, windowCount);

  float meanBFrac =
    mean(windowBFraction, windowCount);

  float meanBOverR =
    mean(windowBOverR, windowCount);


  float sdRFrac =
    standardDeviation(
      windowRFraction,
      windowCount,
      meanRFrac
    );


  float rSum = 0.0;
  float gSum = 0.0;
  float bSum = 0.0;

  float rFracSum = 0.0;
  float bFracSum = 0.0;
  float bOverRSum = 0.0;

  byte accepted = 0;


  // ----------------------------------------------------------
  // FILTRO DA JANELA
  // ----------------------------------------------------------

  for (byte i = 0; i < windowCount; i++) {

    bool keep = true;


    if (sdRFrac > 0.0) {

      keep =
        absFloat(windowRFraction[i] - meanRFrac)
        <= OUTLIER_SIGMA_LIMIT * sdRFrac;
    }


    if (keep) {

      rSum += windowR[i];
      gSum += windowG[i];
      bSum += windowB[i];

      rFracSum += windowRFraction[i];
      bFracSum += windowBFraction[i];
      bOverRSum += windowBOverR[i];

      accepted++;
    }
  }


  // ----------------------------------------------------------
  // SE NENHUMA FOI ACEITA
  // ----------------------------------------------------------

  if (accepted == 0) {

    accepted = windowCount;

    for (byte i = 0; i < windowCount; i++) {

      rSum += windowR[i];
      gSum += windowG[i];
      bSum += windowB[i];

      rFracSum += windowRFraction[i];
      bFracSum += windowBFraction[i];
      bOverRSum += windowBOverR[i];
    }
  }


  // ----------------------------------------------------------
  // RESULTADO
  // ----------------------------------------------------------

  ColorReading result;

  result.r = rSum / accepted;
  result.g = gSum / accepted;
  result.b = bSum / accepted;

  result.rFraction = rFracSum / accepted;
  result.bFraction = bFracSum / accepted;
  result.bOverR = bOverRSum / accepted;


  result.rFractionSD =
    standardDeviation(
      windowRFraction,
      windowCount,
      result.rFraction
    );


  result.bFractionSD =
    standardDeviation(
      windowBFraction,
      windowCount,
      result.bFraction
    );


  result.bOverRSD =
    standardDeviation(
      windowBOverR,
      windowCount,
      result.bOverR
    );


  result.acceptedSamples = accepted;


  return result;
}


// ============================================================
// LEITURA DO CANAL
// ============================================================

float readChannelIntensity(char channel) {

  switch (channel) {

    case 'R':

      digitalWrite(PIN_S2, LOW);
      digitalWrite(PIN_S3, LOW);

      break;


    case 'G':

      digitalWrite(PIN_S2, HIGH);
      digitalWrite(PIN_S3, HIGH);

      break;


    case 'B':

      digitalWrite(PIN_S2, LOW);
      digitalWrite(PIN_S3, HIGH);

      break;
  }


  delayMicroseconds(300);


  unsigned long pulseWidth =
    pulseIn(
      PIN_OUT,
      LOW,
      PULSE_TIMEOUT_US
    );


  if (pulseWidth == 0) {

    return 0.0;
  }


  // Quanto menor o pulso,
  // maior a frequencia e maior o sinal.
  return 1.0 / (float)pulseWidth;
}


// ============================================================
// METRICA R/(R+G+B)
// ============================================================

float calculateRFraction(
  float r,
  float g,
  float b
) {

  float total = r + g + b;


  if (total <= 0.0) {

    return 0.0;
  }


  return r / total;
}


// ============================================================
// METRICA B/(R+G+B)
// ============================================================

float calculateBFraction(
  float r,
  float g,
  float b
) {

  float total = r + g + b;


  if (total <= 0.0) {

    return 0.0;
  }


  return b / total;
}


// ============================================================
// METRICA B/R
// ============================================================

float calculateBOverR(
  float r,
  float b
) {

  if (r <= 0.00000001) {

    return 0.0;
  }


  return b / r;
}


// ============================================================
// RESULTADO DO BRANCO
// ============================================================

void printBlankResult(ColorReading reading) {

  Serial.println();
  Serial.println(F("=========================================="));
  Serial.println(F("       BRANCO DA SESSAO SALVO"));
  Serial.println(F("=========================================="));

  Serial.print(F("R = "));
  Serial.println(reading.r, 8);

  Serial.print(F("G = "));
  Serial.println(reading.g, 8);

  Serial.print(F("B = "));
  Serial.println(reading.b, 8);

  Serial.println();

  Serial.print(F("R/(R+G+B) = "));
  Serial.println(reading.rFraction, 8);

  Serial.print(F("B/(R+G+B) = "));
  Serial.println(reading.bFraction, 8);

  Serial.print(F("B/R = "));
  Serial.println(reading.bOverR, 8);

  Serial.println();

  Serial.println(F("Branco salvo."));
  Serial.println(F("Agora coloque uma amostra e envie OK."));
  Serial.println();
}


// ============================================================
// RESULTADO FINAL DA AMOSTRA
// ============================================================

void printFinalSampleResult(ColorReading reading) {

  Serial.println();
  Serial.println(F("=========================================="));
  Serial.println(F("           RESULTADO DA AMOSTRA"));
  Serial.println(F("=========================================="));

  Serial.println();

  Serial.println(F("--- CANAIS BRUTOS ---"));

  Serial.print(F("R = "));
  Serial.println(reading.r, 8);

  Serial.print(F("G = "));
  Serial.println(reading.g, 8);

  Serial.print(F("B = "));
  Serial.println(reading.b, 8);

  Serial.println();

  Serial.println(F("--- METRICAS DE COR ---"));

  Serial.print(F("R/(R+G+B) = "));
  Serial.println(reading.rFraction, 8);

  Serial.print(F("B/(R+G+B) = "));
  Serial.println(reading.bFraction, 8);

  Serial.print(F("B/R = "));
  Serial.println(reading.bOverR, 8);

  Serial.println();

  Serial.println(F("--- VARIACAO ---"));

  Serial.print(F("SD R/(R+G+B) = "));
  Serial.println(reading.rFractionSD, 8);

  Serial.print(F("SD B/(R+G+B) = "));
  Serial.println(reading.bFractionSD, 8);

  Serial.print(F("SD B/R = "));
  Serial.println(reading.bOverRSD, 8);

  Serial.print(F("Blocos aceitos = "));
  Serial.println(reading.acceptedSamples);

  Serial.println();


  // ----------------------------------------------------------
  // COMPARACAO COM O BRANCO
  // ----------------------------------------------------------

  if (hasSessionBlank) {

    Serial.println(F("--- COMPARACAO COM O BRANCO ---"));

    Serial.print(F("Delta Rfrac = "));
    Serial.println(
      reading.rFraction - blankRFraction,
      8
    );

    Serial.print(F("Delta Bfrac = "));
    Serial.println(
      reading.bFraction - blankBFraction,
      8
    );

    Serial.print(F("Delta B/R = "));
    Serial.println(
      reading.bOverR - blankBOverR,
      8
    );

    Serial.println();
  }


  Serial.println(F("------------------------------------------"));

  Serial.println(
    F("NAO HA CONCENTRACAO ESTIMADA NESTA VERSAO.")
  );

  Serial.println(
    F("Use estes sinais para construir a NOVA curva.")
  );

  Serial.println(F("------------------------------------------"));

  Serial.println();
}


// ============================================================
// MEDIA
// ============================================================

float mean(float values[], byte count) {

  if (count == 0) {

    return 0.0;
  }


  float sum = 0.0;


  for (byte i = 0; i < count; i++) {

    sum += values[i];
  }


  return sum / count;
}


// ============================================================
// DESVIO-PADRAO
// ============================================================

float standardDeviation(
  float values[],
  byte count,
  float avg
) {

  if (count < 2) {

    return 0.0;
  }


  float sumSquares = 0.0;


  for (byte i = 0; i < count; i++) {

    float diff = values[i] - avg;

    sumSquares += diff * diff;
  }


  return sqrt(
    sumSquares / (count - 1)
  );
}


// ============================================================
// VALOR ABSOLUTO
// ============================================================

float absFloat(float value) {

  if (value < 0.0) {

    return -value;
  }


  return value;
}
