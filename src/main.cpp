#include <Arduino.h>
// #include <BluetoothSerial.h>
#include <Adafruit_VL53L0X.h>
#include <Wire.h>
#include <ESP32Servo.h> // Biblioteca para controle de servos

// Definições das portas dos motores
#define FRENTE_D 14
#define TRAS_D 12
#define PWMD 13

#define FRENTE_B 25
#define TRAS_B 26
#define PWMB 27

// Endereços dos sensores VL53L0X
#define LOX1_ADDRESS 0x30
#define LOX2_ADDRESS 0x31
#define LOX3_ADDRESS 0x32
#define LOX4_ADDRESS 0x33
#define LOX5_ADDRESS 0x34

// Pinos de shutdown dos sensores VL53L0X
#define SHT_LOX1 33
#define SHT_LOX2 32
#define SHT_LOX3 19
#define SHT_LOX4 5
#define SHT_LOX5 18

// Pinos dos sensores infravermelhos
#define S0_A 0
#define S1_B 15
#define IR_sense 34
#define STRAT1 35

const int BRANCO = 50;

// Configuração dos motores
const int VELOCIDADE_MIN = 60;
const int VELOCIDADE_MAX = 254;
const int NEUTRO = 127;
const float angular = 1;

int sensorInfraRedData[4];
int sensorLaserData[5];

unsigned long previousMillis = 0;
const long interval = 10;
int contador = 0;

enum estados {
	BUSCA,
	ATAQUE,
	LOCALIZADO
};

int estadoAtual = BUSCA;
unsigned long lastMillis = 0;

// BluetoothSerial SerialBT;
Adafruit_VL53L0X lox1 = Adafruit_VL53L0X();
Adafruit_VL53L0X lox2 = Adafruit_VL53L0X();
Adafruit_VL53L0X lox3 = Adafruit_VL53L0X();
Adafruit_VL53L0X lox4 = Adafruit_VL53L0X();
Adafruit_VL53L0X lox5 = Adafruit_VL53L0X();

Servo servo_dir;
Servo servo_esq;
int posInicial = 90; // Posição inicial dos servos
int posFinalDireita = 175;  // Posição final dos servos
int posFinalEsquerda = 5;  // Posição final dos servos

VL53L0X_RangingMeasurementData_t measure1;
VL53L0X_RangingMeasurementData_t measure2;
VL53L0X_RangingMeasurementData_t measure3;
VL53L0X_RangingMeasurementData_t measure4;
VL53L0X_RangingMeasurementData_t measure5;

void setID() {
    digitalWrite(SHT_LOX1, LOW);
    digitalWrite(SHT_LOX2, LOW);
    digitalWrite(SHT_LOX3, LOW);
    digitalWrite(SHT_LOX4, LOW);
    digitalWrite(SHT_LOX5, LOW);

    digitalWrite(SHT_LOX1, HIGH);
    digitalWrite(SHT_LOX2, HIGH);
    digitalWrite(SHT_LOX3, HIGH);
    digitalWrite(SHT_LOX4, HIGH);
    digitalWrite(SHT_LOX5, HIGH);

    digitalWrite(SHT_LOX1, HIGH);
    digitalWrite(SHT_LOX2, LOW);
    digitalWrite(SHT_LOX3, LOW);
    digitalWrite(SHT_LOX4, LOW);
    digitalWrite(SHT_LOX5, LOW);

    if(!lox1.begin(LOX1_ADDRESS)) {
        Serial.println(F("Failed to boot first VL53L0X"));
        while(1);
    }

    digitalWrite(SHT_LOX2, HIGH);
    if(!lox2.begin(LOX2_ADDRESS)){
        Serial.println(F("Failed to boot second VL53L0X"));
        while(1);
    }

    digitalWrite(SHT_LOX3, HIGH);
    if(!lox3.begin(LOX3_ADDRESS)){
        Serial.println(F("Failed to boot third VL53L0X"));
        while(1);
    }

    digitalWrite(SHT_LOX4, HIGH);
    if(!lox4.begin(LOX4_ADDRESS)){
        Serial.println(F("Failed to boot fourth VL53L0X"));
        while(1);
    }

    digitalWrite(SHT_LOX5, HIGH);
    if(!lox5.begin(LOX5_ADDRESS)){
        Serial.println(F("Failed to boot fifth VL53L0X"));
        while(1);
    }
}

void readSensors() {
    lox1.rangingTest(&measure1, false);
    lox2.rangingTest(&measure2, false);
    lox3.rangingTest(&measure3, false);
    lox4.rangingTest(&measure4, false);
    lox5.rangingTest(&measure5, false);
}

void getSensorData() {
    readSensors();
    int distance1 = (measure1.RangeStatus != 4 && measure1.RangeMilliMeter <= 800) ? measure1.RangeMilliMeter : 800;
    int distance2 = (measure2.RangeStatus != 4 && measure2.RangeMilliMeter <= 800) ? measure2.RangeMilliMeter : 800;
    int distance3 = (measure3.RangeStatus != 4 && measure3.RangeMilliMeter <= 800) ? measure3.RangeMilliMeter : 800;
    int distance4 = (measure4.RangeStatus != 4 && measure4.RangeMilliMeter <= 800) ? measure4.RangeMilliMeter : 800;
    int distance5 = (measure5.RangeStatus != 4 && measure5.RangeMilliMeter <= 800) ? measure5.RangeMilliMeter : 800;
    sensorLaserData[0] = distance3/10;
    sensorLaserData[1] = distance5/10;
    sensorLaserData[2] = distance4/10;
    sensorLaserData[3] = distance1/10;
    sensorLaserData[4] = distance2/10;
}

String formatSensorLaser(){
    return String(sensorLaserData[0] / 10) + " " +
            String(sensorLaserData[1] / 10) + " " +
            String(sensorLaserData[2] / 10) + " " +
            String(sensorLaserData[3] / 10) + " " +
            String(sensorLaserData[4] / 10);
}

void getInfraRedSensorData() {
    digitalWrite(S0_A, LOW);
    digitalWrite(S1_B, LOW);
    sensorInfraRedData[0] = analogRead(IR_sense);

    digitalWrite(S0_A, HIGH);
    digitalWrite(S1_B, LOW);
    sensorInfraRedData[1] = analogRead(IR_sense);

    digitalWrite(S0_A, HIGH);
    digitalWrite(S1_B, HIGH);
    sensorInfraRedData[2] = analogRead(IR_sense);

    digitalWrite(S0_A, LOW);
    digitalWrite(S1_B, HIGH);
    sensorInfraRedData[3] = analogRead(IR_sense);
}

String formatInfraRedSensor() {
    String formatInfraRed = "";
    formatInfraRed += String(sensorInfraRedData[0]) + " ";
    formatInfraRed += String(sensorInfraRedData[1]) + " ";
    formatInfraRed += String(sensorInfraRedData[2]) + " ";
    formatInfraRed += String(sensorInfraRedData[3]) + " ";
    return formatInfraRed;
}

void controleMotores(int VD, int VE) {
    if (VD > 0) {
        digitalWrite(FRENTE_D, HIGH);
        digitalWrite(TRAS_D, LOW);
        analogWrite(PWMD, VD);
    } else if (VD < 0) {
        digitalWrite(FRENTE_D, LOW);
        digitalWrite(TRAS_D, HIGH);
        analogWrite(PWMD, -VD);
    } else {
        digitalWrite(FRENTE_D, LOW);
        digitalWrite(TRAS_D, LOW);
        analogWrite(PWMD, 0);
    }

    if (VE > 0) {
        digitalWrite(FRENTE_B, HIGH);
        digitalWrite(TRAS_B, LOW);
        analogWrite(PWMB, VE);
    } else if (VE < 0) {
        digitalWrite(FRENTE_B, LOW);
        digitalWrite(TRAS_B, HIGH);
        analogWrite(PWMB, -VE);
    } else {
        digitalWrite(FRENTE_B, LOW);
        digitalWrite(TRAS_B, LOW);
        analogWrite(PWMB, 0);
    }
}

void processarEntrada(String entrada) {
    int espacoIndex = entrada.indexOf(' ');
    int V = entrada.substring(0, espacoIndex).toInt();
    int W = entrada.substring(espacoIndex + 1).toInt();

    int VD, VE;

    if (V > NEUTRO) {
        VD = map(V, NEUTRO + 1, 254, VELOCIDADE_MIN, VELOCIDADE_MAX);
        VE = VD;
    } else if (V < NEUTRO) {
        VD = -map(V, 0, NEUTRO - 1, VELOCIDADE_MAX, VELOCIDADE_MIN);
        VE = VD;
    } else {
        VD = 0;
        VE = 0;
    }

    if (W > NEUTRO) {
        VD = VD - map(W, NEUTRO + 1, 254, VELOCIDADE_MIN/angular, VELOCIDADE_MAX/angular);
        VE = VE + map(W, NEUTRO + 1, 254, VELOCIDADE_MIN/angular, VELOCIDADE_MAX/angular);
    } else if (W < NEUTRO) {
        VD = VD + map(W, 0, NEUTRO - 1, VELOCIDADE_MAX/angular, VELOCIDADE_MIN/angular);
        VE = VE - map(W, 0, NEUTRO - 1, VELOCIDADE_MAX/angular, VELOCIDADE_MIN/angular);
    }

    controleMotores(VD, VE);
}

void processarEstrategia(int &velocidadeLinear, int &velocidadeAngular) {
    int RANGE_LASER = 30;

    velocidadeLinear = NEUTRO - 1;
    velocidadeAngular = NEUTRO;
    if (sensorLaserData[3] < RANGE_LASER) { // Inimigo localizado na Lateral Esquerda
		velocidadeLinear = NEUTRO;
        velocidadeAngular = NEUTRO - 45;
    }
    if (sensorLaserData[0] < RANGE_LASER) { // Inimigo localizado na Diagonal Esquerda
		velocidadeLinear = NEUTRO;
        velocidadeAngular = NEUTRO - 45;
    }
    if (sensorLaserData[4] < RANGE_LASER) { // Inimigo localizado na Lateral Direita
		velocidadeLinear = NEUTRO;
        velocidadeAngular = NEUTRO + 50;
    }
    if (sensorLaserData[2] < RANGE_LASER) { // Inimigo localizado na Diagonal Direita
        velocidadeLinear = NEUTRO;
        velocidadeAngular = NEUTRO + 50;
    }
    if (sensorLaserData[1] < RANGE_LASER) { // Inimigo localizado pelo Sensor central
        velocidadeLinear = NEUTRO + 63;
        velocidadeAngular = NEUTRO;
    }
}

void realizarRotacao(){
	// unsigned long currentMillis = millis();
	// while (true) {
		// if (currentMillis - lastMillis < 1500) {
		// 	lastMillis = currentMillis;
		
	processarEntrada("207 127");
	delay(300);
			// continue;
		// }
	processarEntrada("127 254");
	delay(200);
	// 	break;
	// }
}

void readSensorsTask(void *parameter) {
    while (true) {
        getSensorData();
    }
}

void controlMotorsTask(void *parameter) {
    int V = NEUTRO, W = NEUTRO;

    while (true) {
		getInfraRedSensorData();
        bool limitDetected = false;

        if (sensorInfraRedData[0] < BRANCO || sensorInfraRedData[3] < BRANCO) { // Direita frente e Esquerda frente
            limitDetected = true;
            V = NEUTRO - 80;
            W = NEUTRO;
			// realizarRotacao();
        }
        if (sensorInfraRedData[2] < BRANCO || sensorInfraRedData[3] < BRANCO) { // Direita trás e Esquerda trás
            limitDetected = true;
            V = NEUTRO + 80;
            W = NEUTRO;
			realizarRotacao();
			continue;
        }
        if (sensorInfraRedData[0] < BRANCO && sensorInfraRedData[1] < BRANCO && sensorInfraRedData[2] < BRANCO && sensorInfraRedData[3] < BRANCO) { // Todos os sensores
            limitDetected = true;
            V = NEUTRO;
            W = NEUTRO;
        }
        
        if (!limitDetected) {
            processarEstrategia(V, W);
        }

        String comando = String(V) + " " + String(W);
        processarEntrada(comando);
    }
}

void setup() {
    Serial.begin(115200);

    pinMode(FRENTE_D, OUTPUT);
    pinMode(TRAS_D, OUTPUT);
    pinMode(PWMD, OUTPUT);
    pinMode(FRENTE_B, OUTPUT);
    pinMode(TRAS_B, OUTPUT);
    pinMode(PWMB, OUTPUT);

    pinMode(SHT_LOX1, OUTPUT);
    pinMode(SHT_LOX2, OUTPUT);
    pinMode(SHT_LOX3, OUTPUT);
    pinMode(SHT_LOX4, OUTPUT);
    pinMode(SHT_LOX5, OUTPUT);

    pinMode(S0_A, OUTPUT);
    pinMode(S1_B, OUTPUT);
    pinMode(IR_sense, INPUT);
    pinMode(STRAT1, INPUT);

    setID();

    servo_dir.attach(4); // Pino do servo direito
    servo_esq.attach(16); // Pino do servo esquerdo
    servo_dir.write(posInicial); // Inicializa na posição inicial
    servo_esq.write(posInicial); // Inicializa na posição inicial

    // Cria as tarefas fixadas em núcleos específicos
    xTaskCreatePinnedToCore(
        readSensorsTask,   // Função de tarefa
        "ReadSensors",     // Nome da tarefa
        10000,             // Tamanho da pilha
        NULL,              // Parâmetro passado para a função da tarefa
        1,                 // Prioridade da tarefa
        NULL,              // Handle da tarefa (não utilizado)
        0                  // Núcleo no qual a tarefa será fixada
    );

    xTaskCreatePinnedToCore(
        controlMotorsTask, // Função de tarefa
        "ControlMotors",   // Nome da tarefa
        10000,             // Tamanho da pilha
        NULL,              // Parâmetro passado para a função da tarefa
        1,                 // Prioridade da tarefa
        NULL,              // Handle da tarefa (não utilizado)
        1                  // Núcleo no qual a tarefa será fixada
    );
}

void loop() {
    // O loop vazio, pois as tarefas são executadas nos núcleos
}